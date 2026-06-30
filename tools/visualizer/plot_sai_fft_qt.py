"""
PCM1840 4CH 실시간 FFT / dB 도메인 시각화 (pyqtgraph)

기존 plot_sai_test.py(matplotlib + ASCII RAW)의 오버헤드를 줄이기 위한 버전.

핵심 차이:
  1) 시리얼: ASCII 라인 파싱 대신 12바이트 바이너리 프레임을 bulk read +
     numpy 벡터 파싱. (펌웨어 STREAM_BINARY = true 필요)
  2) GUI: matplotlib 전체 리드로우 대신 pyqtgraph로 라인/이미지 아이템만 갱신.
  3) STFT: 채널 4개를 한 번에 벡터화하여 hop 단위로 정확히 프레이밍.

펌웨어 프레임 포맷 (Little-Endian, 12 byte):
  [0xA5][0x5A]  magic
  [seq  u16]
  [ch0..ch3 i16]
"""

import sys
import time
from collections import deque
from threading import Thread, Event, Lock

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# =========================================================
# Serial 설정
# =========================================================

# SERIAL_PORT = "/dev/cu.usbmodem1101"   # macOS
SERIAL_PORT = "COM3"                      # Windows
BAUD_RATE = 2000000

# =========================================================
# 오디오 / FFT 설정
# =========================================================

CHANNEL_COUNT = 4

# 펌웨어 RAW_DECIMATION 과 반드시 일치해야 함. (TDOA 위해 48kHz 풀레이트)
#   RAW_DECIMATION = 1  -> 48000 Hz       (관측 ~8000 Hz, Nyquist 24000)
#   RAW_DECIMATION = 3  -> 48000/3 = 16000 Hz
SAMPLE_RATE = 48000

# 관측할 최대 주파수. Nyquist(SAMPLE_RATE/2) 이하로 둘 것.
MAX_FREQ = 8000

# 48kHz 라 bin 폭 유지를 위해 FFT_SIZE 상향 (2048 -> bin 23.4Hz, frame 42.7ms)
FFT_SIZE = 2048
HOP_SIZE = 512
HISTORY_FRAMES = 100

DB_FLOOR = -120.0
DB_CEIL = 0.0

# GUI 갱신 주기 (ms)
GUI_INTERVAL_MS = 30

# =========================================================
# 바이너리 프레임 정의
# =========================================================

MAGIC = 0x5AA5  # 바이트 A5 5A 의 LE uint16
FRAME_DTYPE = np.dtype([
    ("magic", "<u2"),
    ("seq", "<u2"),
    ("ch", "<i2", (CHANNEL_COUNT,)),
])
FRAME_BYTES = FRAME_DTYPE.itemsize  # 12

# =========================================================
# 공유 버퍼 (reader -> GUI)
# =========================================================

stop_event = Event()
sample_blocks = deque()          # 각 원소: (M, CHANNEL_COUNT) float32
stats_lock = Lock()
stats = {"rx_rate": 0.0, "lost": 0}

# =========================================================
# FFT 준비
# =========================================================

freqs = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)
freq_mask = freqs <= MAX_FREQ
freqs_view = freqs[freq_mask]

window = np.hanning(FFT_SIZE).astype(np.float32)

# signed32 >> 16 으로 줄인 16-bit full scale 기준 dBFS
fft_ref = (np.sum(window) / 2.0) * 32768.0


# =========================================================
# Serial 수신 스레드 (바이너리 bulk read + 벡터 파싱)
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
    last_seq = None

    meter_count = 0
    meter_start = time.time()

    while not stop_event.is_set():
        try:
            chunk = ser.read(4096)
        except Exception:
            continue

        if chunk:
            buf.extend(chunk)

        # ---- magic 정렬 ----
        if not synced:
            idx = buf.find(b"\xA5\x5A")
            if idx < 0:
                # magic 후보가 없음. 끝의 1바이트만 남기고 버림(분할 magic 대비)
                if len(buf) > 1:
                    del buf[:-1]
                continue
            del buf[:idx]
            if len(buf) < 2 * FRAME_BYTES:
                continue  # 다음 프레임 검증까지 더 받기
            # 다음 프레임도 magic 이면 정렬 확정
            if buf[FRAME_BYTES] == 0xA5 and buf[FRAME_BYTES + 1] == 0x5A:
                synced = True
            else:
                del buf[:1]  # 가짜 magic. 한 바이트 밀고 재탐색
                continue

        # ---- 완성된 프레임 일괄 파싱 ----
        n = len(buf) // FRAME_BYTES
        if n == 0:
            continue

        raw = bytes(buf[:n * FRAME_BYTES])
        del buf[:n * FRAME_BYTES]

        arr = np.frombuffer(raw, dtype=FRAME_DTYPE, count=n)

        if not np.all(arr["magic"] == MAGIC):
            # 동기 깨짐. 정렬부터 다시.
            synced = False
            continue

        samples = arr["ch"].astype(np.float32)  # (n, CHANNEL_COUNT)
        sample_blocks.append(samples)

        # ---- 손실 프레임 추정 (seq 연속성, u16 wrap 고려) ----
        seq_arr = arr["seq"].astype(np.int64)
        block_lost = 0
        if last_seq is not None and n > 0:
            gap0 = (int(seq_arr[0]) - last_seq - 1) & 0xFFFF
            block_lost += gap0
        if n > 1:
            diffs = (np.diff(seq_arr) - 1) & 0xFFFF
            block_lost += int(diffs.sum())
        last_seq = int(seq_arr[-1])

        meter_count += n
        now = time.time()
        if now - meter_start >= 1.0:
            with stats_lock:
                stats["rx_rate"] = meter_count / (now - meter_start)
            meter_count = 0
            meter_start = now

        if block_lost:
            with stats_lock:
                stats["lost"] += block_lost

    ser.close()
    print("Serial closed.")


# =========================================================
# pyqtgraph GUI
# =========================================================

pg.setConfigOptions(antialias=False, useOpenGL=False, background="k", foreground="w")

app = pg.mkQApp("PCM1840 4CH FFT / dB Domain")

win = pg.GraphicsLayoutWidget(show=True, title="PCM1840 4CH FFT / dB Domain")
win.resize(1400, 900)

# 컬러맵 LUT (스펙트로그램용)
try:
    cmap = pg.colormap.get("inferno")
except Exception:
    try:
        cmap = pg.colormap.get("viridis")
    except Exception:
        cmap = None
lut = cmap.getLookupTable(0.0, 1.0, 256) if cmap is not None else None

time_span = HISTORY_FRAMES * HOP_SIZE / SAMPLE_RATE

spectrum_curves = []
spectrum_plots = []
spec_images = []
# 스펙트로그램 링버퍼: (HISTORY_FRAMES, n_freq_view) -> 축0=시간(x), 축1=주파수(y)
n_fview = len(freqs_view)
spec_ring = [np.full((HISTORY_FRAMES, n_fview), DB_FLOOR, dtype=np.float32)
             for _ in range(CHANNEL_COUNT)]
spec_ptr = [0 for _ in range(CHANNEL_COUNT)]

# 채널별 STFT 입력 버퍼 (GUI 스레드 전용)
pbuf = [np.zeros((0, CHANNEL_COUNT), dtype=np.float32)]  # 단일 공유 배열

for ch in range(CHANNEL_COUNT):
    # 왼쪽: 실시간 스펙트럼
    p_spec = win.addPlot(row=ch, col=0)
    p_spec.setXRange(0, MAX_FREQ, padding=0)
    p_spec.setYRange(DB_FLOOR, 5, padding=0)
    p_spec.showGrid(x=True, y=True, alpha=0.3)
    p_spec.setLabel("left", f"CH{ch + 1} [dBFS]")
    if ch == CHANNEL_COUNT - 1:
        p_spec.setLabel("bottom", "Frequency [Hz]")
    p_spec.setTitle(f"CH{ch + 1} Spectrum")
    curve = p_spec.plot(freqs_view,
                        np.full(n_fview, DB_FLOOR, dtype=np.float32),
                        pen=pg.mkPen((0, 200, 255), width=1))
    spectrum_curves.append(curve)
    spectrum_plots.append(p_spec)

    # 오른쪽: 스펙트로그램
    p_img = win.addPlot(row=ch, col=1)
    p_img.setLabel("left", f"CH{ch + 1} [Hz]")
    if ch == CHANNEL_COUNT - 1:
        p_img.setLabel("bottom", "Time [s]")
    p_img.setTitle(f"CH{ch + 1} Spectrogram")
    img = pg.ImageItem()
    if lut is not None:
        img.setLookupTable(lut)
    img.setLevels((DB_FLOOR, DB_CEIL))
    p_img.addItem(img)
    p_img.setYRange(0, MAX_FREQ, padding=0)
    # 배열[x=time, y=freq] -> (-time_span..0, freqs_view[0]..freqs_view[-1])
    img.setRect(QtCore.QRectF(-time_span, float(freqs_view[0]),
                              time_span, float(freqs_view[-1] - freqs_view[0])))
    spec_images.append(img)

win.ci.layout.setColumnStretchFactor(0, 1)
win.ci.layout.setColumnStretchFactor(1, 1)


def compute_frames(frame_block):
    """(F, CHANNEL_COUNT) float32 윈도우들에 대한 dBFS 반환 (n_fview, CH)."""
    x = frame_block - frame_block.mean(axis=0, keepdims=True)  # DC 제거
    x = x * window[:, None]
    spectrum = np.fft.rfft(x, axis=0)
    magnitude = np.abs(spectrum)
    dbfs = 20.0 * np.log10((magnitude / fft_ref) + 1e-12)
    np.clip(dbfs, DB_FLOOR, DB_CEIL, out=dbfs)
    return dbfs[freq_mask]  # (n_fview, CHANNEL_COUNT)


def update():
    if stop_event.is_set() or not win.isVisible():
        return

    # ---- reader 가 쌓은 블록을 모두 흡수 ----
    new_chunks = []
    while sample_blocks:
        try:
            new_chunks.append(sample_blocks.popleft())
        except IndexError:
            break

    if new_chunks:
        buf = pbuf[0]
        pbuf[0] = np.concatenate([buf] + new_chunks, axis=0)

    buf = pbuf[0]

    latest_db = None      # (n_fview, CH) 마지막 스펙트럼
    new_columns = 0       # 이번 틱에 추가된 스펙트로그램 컬럼 수
    pos = 0
    while pos + FFT_SIZE <= buf.shape[0]:
        frame = buf[pos:pos + FFT_SIZE, :]       # (FFT_SIZE, CH)
        db = compute_frames(frame)               # (n_fview, CH)
        for ch in range(CHANNEL_COUNT):
            ptr = spec_ptr[ch]
            spec_ring[ch][ptr, :] = db[:, ch]
            spec_ptr[ch] = (ptr + 1) % HISTORY_FRAMES
        latest_db = db
        new_columns += 1
        pos += HOP_SIZE

    # 소비한 만큼 버리고 overlap 잔여분 유지
    if pos > 0:
        pbuf[0] = buf[pos:, :]

    # ---- 화면 갱신 ----
    if latest_db is not None:
        for ch in range(CHANNEL_COUNT):
            spectrum_curves[ch].setData(freqs_view, latest_db[:, ch])

            # 피크(DC 제외)
            col = latest_db[:, ch]
            peak_idx = int(np.argmax(col[1:])) + 1 if n_fview > 1 else 0
            spectrum_plots[ch].setTitle(
                f"CH{ch + 1} Spectrum | "
                f"Peak {freqs_view[peak_idx]:.0f} Hz, {col[peak_idx]:.1f} dBFS"
            )

        if new_columns:
            for ch in range(CHANNEL_COUNT):
                ptr = spec_ptr[ch]
                ordered = np.concatenate(
                    [spec_ring[ch][ptr:], spec_ring[ch][:ptr]], axis=0)
                spec_images[ch].setImage(ordered, autoLevels=False,
                                         levels=(DB_FLOOR, DB_CEIL))

    with stats_lock:
        rx = stats["rx_rate"]
        lost = stats["lost"]
    win.setWindowTitle(
        f"PCM1840 4CH FFT / dB Domain | "
        f"RX {rx:.0f} samples/s | Target {SAMPLE_RATE} | Lost {lost}"
    )


# =========================================================
# 실행
# =========================================================

reader_thread = Thread(target=serial_reader, daemon=True)
reader_thread.start()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(GUI_INTERVAL_MS)


def on_quit():
    stop_event.set()
    reader_thread.join(timeout=1.0)


app.aboutToQuit.connect(on_quit)

print("Reading serial (binary)...")
print("Close window or Ctrl+C to stop.")

if __name__ == "__main__":
    try:
        pg.exec()
    except KeyboardInterrupt:
        stop_event.set()
