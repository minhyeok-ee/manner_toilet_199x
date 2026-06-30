"""
4-mic TDOA 음원 위치 추정 프로토타입 (GCC-PHAT + SRP-PHAT)

펌웨어 STREAM_BINARY(48kHz, RAW_DECIMATION=1) 바이너리 스트림을 그대로 읽어
사각형 모서리 4-mic 으로 음원의 (x, y) 위치를 근거리(near-field)에서 추정한다.

파이프라인:
  1) 12바이트 바이너리 프레임 bulk read (plot_sai_fft_qt.py 와 동일 프로토콜)
  2) 짧은 윈도우(WINDOW) 단위로 에너지 게이팅 -> 소리 있을 때만 위치 계산
  3) 마이크 쌍별 GCC-PHAT 상호상관 (PHAT 가중 -> 잔향에 강함)
  4) SRP-PHAT: 후보 격자점마다 이론 TDOA 위치의 상관값을 합산 -> 최댓값이 음원
  5) pyqtgraph 로 SRP 파워 히트맵 + 마이크/추정 위치 표시

좌표계: 어레이 중심이 원점. 단위 meter. 모든 마이크가 z=0 평면에 있다고 가정(2D).
"""

import time
from collections import deque
from threading import Thread, Event

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# =========================================================
# Serial / 프레임 (plot_sai_fft_qt.py 와 동일)
# =========================================================

SERIAL_PORT = "COM3"
BAUD_RATE = 2000000

CHANNEL_COUNT = 4
SAMPLE_RATE = 48000               # 펌웨어 RAW_DECIMATION=1 과 일치

MAGIC = 0x5AA5
FRAME_DTYPE = np.dtype([
    ("magic", "<u2"),
    ("seq", "<u2"),
    ("ch", "<i2", (CHANNEL_COUNT,)),
])
FRAME_BYTES = FRAME_DTYPE.itemsize

# =========================================================
# 어레이 기하 — 21cm(x) x 13cm(y), 중심 원점
#
#   M4(-.105,+.065) ---- M3(+.105,+.065)
#        |                     |
#   M1(-.105,-.065) ---- M2(+.105,-.065)
#
# !! 채널 순서 검증 필요 !!
# 아래는 CH1->M1, CH2->M2, CH3->M3, CH4->M4 가정.
# 실제 배선이 다르면 이 배열의 행 순서를 바꿔라.
# =========================================================

C_SOUND = 343.0  # m/s (20도 기준; 필요시 보정)

# 어레이 크기: 대각선 40cm, 종횡비 ~1.4:1 (변기 림 ergonomics + 기존 21:13 절충).
#   CRB상 음원 40cm에서 방위~0.08cm, 거리~0.84cm (평면 내).
#   ** 긴 축(y)을 변기 앞-뒤(전후방)로 정렬할 것 — 음원 방위 방향 분해능↑ **
ARRAY_W = 0.233    # x 좌우(side-to-side) [m]
ARRAY_H = 0.326    # y 전후방(front-back, 긴 축) [m]  -> 대각선 √(W²+H²) ≈ 0.40

# !! 채널 순서 검증 필요 !! CH1->좌하, CH2->우하, CH3->우상, CH4->좌상 가정.
# 실제 배선이 다르면 행 순서를 바꿔라.
MIC_POS = np.array([
    [-ARRAY_W / 2, -ARRAY_H / 2],   # CH1
    [+ARRAY_W / 2, -ARRAY_H / 2],   # CH2
    [+ARRAY_W / 2, +ARRAY_H / 2],   # CH3
    [-ARRAY_W / 2, +ARRAY_H / 2],   # CH4
], dtype=np.float64)

# 마이크 쌍 (6개 전부 사용 -> SRP-PHAT 강건성)
PAIRS = [(i, j) for i in range(CHANNEL_COUNT) for j in range(i + 1, CHANNEL_COUNT)]

# =========================================================
# TDOA / SRP-PHAT 파라미터
# =========================================================

WINDOW = 2048                      # 위치 계산 윈도우 (~42.7ms @48k)
NFFT = 1 << int(np.ceil(np.log2(2 * WINDOW)))  # 4096

# 최대 가능한 지연(샘플): 대각선 / c * fs + 여유
_max_baseline = np.max([
    np.linalg.norm(MIC_POS[i] - MIC_POS[j]) for i, j in PAIRS
])
MAX_LAG = int(np.ceil(_max_baseline / C_SOUND * SAMPLE_RATE)) + 2
LAGS = np.arange(-MAX_LAG, MAX_LAG + 1)        # 정수 지연 인덱스

# GCC-PHAT 사용 주파수 대역.
# 40cm 어레이는 간격이 커서 고주파 공간 에일리어싱(SRP 부엽) 위험 -> 8kHz 로 제한.
# (λ/2 = 2.1cm @8kHz; 광대역 과도음이면 큰 구경에도 상관 피크는 모호하지 않음.
#  부엽이 보이면 더 낮춰라. 협대역/톤 음원이면 특히 8kHz 이하 유지.)
FMIN, FMAX = 200.0, 8000.0
_rfft_freqs = np.fft.rfftfreq(NFFT, d=1.0 / SAMPLE_RATE)
BAND_MASK = ((_rfft_freqs >= FMIN) & (_rfft_freqs <= FMAX)).astype(np.float64)

# 탐색 격자 (어레이 주변 ±0.35m, 1cm 간격). 소스가 평면(z=0)에 있다고 가정.
GRID_MIN, GRID_MAX, GRID_STEP = -0.35, 0.35, 0.01
gx = np.arange(GRID_MIN, GRID_MAX + 1e-9, GRID_STEP)
gy = np.arange(GRID_MIN, GRID_MAX + 1e-9, GRID_STEP)
GX, GY = np.meshgrid(gx, gy, indexing="xy")    # (ny, nx)
GRID_PTS = np.stack([GX.ravel(), GY.ravel()], axis=1)  # (Npts, 2)

# 검출 게이트: 윈도우 RMS 가 이 값 이상일 때만 위치 계산 (콘솔 RMS 보고 튜닝)
ENERGY_THRESHOLD = 80.0

# =========================================================
# 격자 -> 쌍별 이론 TDOA(샘플) 테이블 (기하 고정이므로 1회만 계산)
#   tau_ij = (dist(p, mic_i) - dist(p, mic_j)) / c * fs
# =========================================================

def build_tau_table():
    # 각 마이크까지 거리: (Npts, CH)
    d = np.linalg.norm(GRID_PTS[:, None, :] - MIC_POS[None, :, :], axis=2)
    tau = np.empty((len(PAIRS), GRID_PTS.shape[0]), dtype=np.float64)
    for k, (i, j) in enumerate(PAIRS):
        tau[k] = (d[:, i] - d[:, j]) / C_SOUND * SAMPLE_RATE
    return tau

TAU_TABLE = build_tau_table()      # (Npairs, Npts) 샘플 단위 지연

# =========================================================
# 공유 버퍼
# =========================================================

stop_event = Event()
sample_blocks = deque()

# =========================================================
# Serial reader (바이너리, plot_sai_fft_qt.py 와 동일 로직)
# =========================================================

def serial_reader():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
        print("Serial connected:", SERIAL_PORT)
    except Exception as e:
        print("Serial open failed:", e)
        stop_event.set()
        return

    buf = bytearray()
    synced = False

    while not stop_event.is_set():
        try:
            chunk = ser.read(8192)
        except Exception:
            continue
        if chunk:
            buf.extend(chunk)

        if not synced:
            idx = buf.find(b"\xA5\x5A")
            if idx < 0:
                if len(buf) > 1:
                    del buf[:-1]
                continue
            del buf[:idx]
            if len(buf) < 2 * FRAME_BYTES:
                continue
            if buf[FRAME_BYTES] == 0xA5 and buf[FRAME_BYTES + 1] == 0x5A:
                synced = True
            else:
                del buf[:1]
                continue

        n = len(buf) // FRAME_BYTES
        if n == 0:
            continue
        raw = bytes(buf[:n * FRAME_BYTES])
        del buf[:n * FRAME_BYTES]

        arr = np.frombuffer(raw, dtype=FRAME_DTYPE, count=n)
        if not np.all(arr["magic"] == MAGIC):
            synced = False
            continue

        sample_blocks.append(arr["ch"].astype(np.float32))

    ser.close()
    print("Serial closed.")

# =========================================================
# GCC-PHAT + SRP-PHAT
# =========================================================

def srp_phat(window_samples):
    """(WINDOW, CH) -> (srp_map (ny,nx), best_xy, peak_val)."""
    x = window_samples - window_samples.mean(axis=0, keepdims=True)  # DC 제거
    # 채널별 rfft (1회)
    X = np.fft.rfft(x * np.hanning(window_samples.shape[0])[:, None],
                    n=NFFT, axis=0)               # (NFFT/2+1, CH)

    srp = np.zeros(GRID_PTS.shape[0], dtype=np.float64)
    for k, (i, j) in enumerate(PAIRS):
        R = X[:, i] * np.conj(X[:, j])
        R /= np.abs(R) + 1e-9                      # PHAT 가중
        R *= BAND_MASK                             # 대역제한(에일리어싱 부엽 억제)
        cc = np.fft.irfft(R, n=NFFT)
        # irfft 결과는 순환 -> 음/양 지연을 LAGS 범위로 재배열
        cc = np.concatenate((cc[-MAX_LAG:], cc[:MAX_LAG + 1]))  # len = 2*MAX_LAG+1
        # 격자 이론 지연에서 상관값 보간 후 합산
        srp += np.interp(TAU_TABLE[k], LAGS, cc)

    best = int(np.argmax(srp))
    return srp.reshape(GX.shape), GRID_PTS[best], float(srp[best])

# =========================================================
# pyqtgraph GUI
# =========================================================

pg.setConfigOptions(antialias=False, background="k", foreground="w")
app = pg.mkQApp("4-mic TDOA Localization (SRP-PHAT)")
win = pg.GraphicsLayoutWidget(show=True, title="4-mic TDOA Localization")
win.resize(760, 760)

plot = win.addPlot()
plot.setAspectLocked(True)
plot.setXRange(GRID_MIN, GRID_MAX)
plot.setYRange(GRID_MIN, GRID_MAX)
plot.setLabel("bottom", "X [m]")
plot.setLabel("left", "Y [m]")

heat = pg.ImageItem()
try:
    heat.setLookupTable(pg.colormap.get("inferno").getLookupTable(0.0, 1.0, 256))
except Exception:
    pass
# ImageItem[x,y] -> 격자 좌표로 매핑
heat.setRect(QtCore.QRectF(GRID_MIN, GRID_MIN,
                           GRID_MAX - GRID_MIN, GRID_MAX - GRID_MIN))
plot.addItem(heat)

mic_scatter = pg.ScatterPlotItem(
    x=MIC_POS[:, 0], y=MIC_POS[:, 1], size=14,
    pen=pg.mkPen("w"), brush=pg.mkBrush(0, 200, 255, 200), symbol="s")
plot.addItem(mic_scatter)
for idx, (mx, my) in enumerate(MIC_POS):
    t = pg.TextItem(f"CH{idx + 1}", color="w", anchor=(0.5, 1.2))
    t.setPos(mx, my)
    plot.addItem(t)

src_scatter = pg.ScatterPlotItem(size=22, pen=pg.mkPen("y", width=2),
                                 brush=pg.mkBrush(255, 255, 0, 0), symbol="o")
plot.addItem(src_scatter)

# 최신 샘플 보관용 롤링 버퍼
pbuf = [np.zeros((0, CHANNEL_COUNT), dtype=np.float32)]


def update():
    if stop_event.is_set() or not win.isVisible():
        return

    new_chunks = []
    while sample_blocks:
        try:
            new_chunks.append(sample_blocks.popleft())
        except IndexError:
            break
    if new_chunks:
        pbuf[0] = np.concatenate([pbuf[0]] + new_chunks, axis=0)[-(WINDOW * 2):]

    buf = pbuf[0]
    if buf.shape[0] < WINDOW:
        return

    w = buf[-WINDOW:, :]
    rms = float(np.sqrt(np.mean(w.astype(np.float64) ** 2)))

    if rms < ENERGY_THRESHOLD:
        win.setWindowTitle(f"Localization | idle (RMS {rms:.1f} < {ENERGY_THRESHOLD})")
        src_scatter.setData([], [])
        return

    srp_map, xy, peak = srp_phat(w)

    # 정규화 히트맵 (표시는 [x,y] 축이라 전치)
    m = srp_map - srp_map.min()
    if m.max() > 0:
        m = m / m.max()
    heat.setImage(m.T, autoLevels=False, levels=(0.0, 1.0))

    src_scatter.setData([xy[0]], [xy[1]])
    win.setWindowTitle(
        f"Localization | src=({xy[0]*100:+.1f}, {xy[1]*100:+.1f}) cm | "
        f"RMS {rms:.0f} | peak {peak:.2f}"
    )
    print(f"src=({xy[0]*100:+6.1f}, {xy[1]*100:+6.1f}) cm  RMS={rms:7.0f}  peak={peak:.3f}")


# =========================================================
# 실행
# =========================================================

reader_thread = Thread(target=serial_reader, daemon=True)
reader_thread.start()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)   # 20 Hz 위치 갱신


def on_quit():
    stop_event.set()
    reader_thread.join(timeout=1.0)


app.aboutToQuit.connect(on_quit)

print(f"TDOA localize: pairs={len(PAIRS)}, grid={GRID_PTS.shape[0]} pts, "
      f"MAX_LAG={MAX_LAG} samp")
print("Make a sound near the array. Ctrl+C or close window to stop.")

if __name__ == "__main__":
    try:
        pg.exec()
    except KeyboardInterrupt:
        stop_event.set()
