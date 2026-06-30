"""
라이브 TDOA 뷰어 — 보드(M7)가 온디바이스로 계산한 위치 결과를 받아 표시만 한다.

펌웨어 LIVE_TDOA 모드의 결과 패킷 (Little-Endian, 12B):
  [0xC3][0x3C] magic
  [seq   u16]
  [x_mm  i16]  음원 x [mm]
  [y_mm  i16]  음원 y [mm]
  [rms   u16]  윈도우 RMS
  [pwr   u16]  SRP 피크*100 (0 = 무음)

PC 는 FFT/SRP 계산을 하지 않는다 — 그냥 받은 (x,y) 를 그린다.
"""

import time
from collections import deque
from threading import Thread, Event

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

# =========================================================
# 설정
# =========================================================

SERIAL_PORT = "COM3"
BAUD_RATE = 2000000

# 어레이 기하 (펌웨어 tdoa.c 와 일치) — 표시용
ARR_W, ARR_H = 0.233, 0.326
MIC = np.array([
    [-ARR_W/2, -ARR_H/2],
    [ ARR_W/2, -ARR_H/2],
    [ ARR_W/2,  ARR_H/2],
    [-ARR_W/2,  ARR_H/2],
], dtype=np.float64)

VIEW = 0.35          # 표시 범위 ±VIEW [m]
PWR_VALID = 1        # pwr >= 이 값이면 유효 위치 (펌웨어가 무음 시 0 전송)
TRAIL = 12           # 위치 잔상 개수

MAGIC = 0x3CC3       # 바이트 C3 3C 의 LE uint16
PKT_DTYPE = np.dtype([
    ("magic", "<u2"),
    ("seq", "<u2"),
    ("x_mm", "<i2"),
    ("y_mm", "<i2"),
    ("rms", "<u2"),
    ("pwr", "<u2"),
])
PKT_BYTES = PKT_DTYPE.itemsize   # 12

# =========================================================
# 공유 상태
# =========================================================

stop_event = Event()
latest = {"x": 0.0, "y": 0.0, "rms": 0, "pwr": 0, "n": 0}

# =========================================================
# Serial reader (결과 패킷 파싱)
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
            chunk = ser.read(4096)
        except Exception:
            continue
        if chunk:
            buf.extend(chunk)

        if not synced:
            idx = buf.find(b"\xC3\x3C")
            if idx < 0:
                if len(buf) > 1:
                    del buf[:-1]
                continue
            del buf[:idx]
            if len(buf) < 2 * PKT_BYTES:
                continue
            if buf[PKT_BYTES] == 0xC3 and buf[PKT_BYTES + 1] == 0x3C:
                synced = True
            else:
                del buf[:1]
                continue

        n = len(buf) // PKT_BYTES
        if n == 0:
            continue
        raw = bytes(buf[:n * PKT_BYTES])
        del buf[:n * PKT_BYTES]
        arr = np.frombuffer(raw, dtype=PKT_DTYPE, count=n)
        if not np.all(arr["magic"] == MAGIC):
            synced = False
            continue

        last = arr[-1]
        latest["x"] = int(last["x_mm"]) / 1000.0
        latest["y"] = int(last["y_mm"]) / 1000.0
        latest["rms"] = int(last["rms"])
        latest["pwr"] = int(last["pwr"])
        latest["n"] += n

    ser.close()
    print("Serial closed.")

# =========================================================
# GUI
# =========================================================

pg.setConfigOptions(antialias=True, background="k", foreground="w")
app = pg.mkQApp("Live TDOA (on-device)")
win = pg.GraphicsLayoutWidget(show=True, title="Live TDOA — on-device localization")
win.resize(760, 760)

plot = win.addPlot()
plot.setAspectLocked(True)
plot.setXRange(-VIEW, VIEW)
plot.setYRange(-VIEW, VIEW)
plot.showGrid(x=True, y=True, alpha=0.3)
plot.setLabel("bottom", "X [m]")
plot.setLabel("left", "Y [m]")

# 마이크 (사각형) + 어레이 외곽
mic_scatter = pg.ScatterPlotItem(
    x=MIC[:, 0], y=MIC[:, 1], size=16,
    pen=pg.mkPen("w"), brush=pg.mkBrush(0, 200, 255, 220), symbol="s")
plot.addItem(mic_scatter)
rect = np.vstack([MIC, MIC[0]])
plot.addItem(pg.PlotDataItem(rect[:, 0], rect[:, 1],
                             pen=pg.mkPen((0, 200, 255, 90), width=1)))
for idx, (mx, my) in enumerate(MIC):
    t = pg.TextItem(f"CH{idx + 1}", color="w", anchor=(0.5, 1.3))
    t.setPos(mx, my)
    plot.addItem(t)

# 위치 잔상 + 현재 마커
trail = deque(maxlen=TRAIL)
trail_scatter = pg.ScatterPlotItem(size=10, pen=None, brush=pg.mkBrush(255, 80, 0, 90))
plot.addItem(trail_scatter)
src_scatter = pg.ScatterPlotItem(size=26, pen=pg.mkPen("y", width=3),
                                 brush=pg.mkBrush(255, 230, 0, 60), symbol="o")
plot.addItem(src_scatter)


def update():
    if stop_event.is_set() or not win.isVisible():
        return
    x, y = latest["x"], latest["y"]
    rms, pwr = latest["rms"], latest["pwr"]

    if pwr >= PWR_VALID:
        trail.append((x, y))
        if trail:
            tx, ty = zip(*trail)
            trail_scatter.setData(tx, ty)
        src_scatter.setData([x], [y])
        win.setWindowTitle(
            f"Live TDOA | src=({x*100:+.1f}, {y*100:+.1f}) cm | "
            f"rms {rms} | pwr {pwr/100:.2f} | pkts {latest['n']}")
    else:
        src_scatter.setData([], [])
        win.setWindowTitle(
            f"Live TDOA | listening... | rms {rms} (thr below) | pkts {latest['n']}")


# =========================================================
# 실행
# =========================================================

reader_thread = Thread(target=serial_reader, daemon=True)
reader_thread.start()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(33)


def on_quit():
    stop_event.set()
    reader_thread.join(timeout=1.0)


app.aboutToQuit.connect(on_quit)

print("Live TDOA viewer. 보드 근처에서 소리를 내면 위치 마커가 움직입니다.")
print("listening 만 뜨면 펌웨어 TDOA_RMS_THR 또는 PC PWR_VALID 를 조정하세요.")

if __name__ == "__main__":
    try:
        pg.exec()
    except KeyboardInterrupt:
        stop_event.set()
