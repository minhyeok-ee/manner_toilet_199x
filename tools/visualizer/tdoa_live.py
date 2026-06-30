"""
라이브 TDOA 정밀 측정 뷰어.

보드(M7)가 ±10cm 격자에서 SRP-PHAT(+포물선 보간)으로 진원지를 계산해 보낸다.
PC 는: 현재 추정을 항상 표시 + 최근 프레임 '중앙값'으로 정밀 측정(±std) +
       검출을 세기별 색으로 누적 로그 + 15cm 관심원 표시.

결과 패킷 (Little-Endian, 12B):
  [0xC3][0x3C] magic
  [seq   u16]
  [x_mm  i16]  진원지 x [mm]
  [y_mm  i16]  진원지 y [mm]
  [rms   u16]  세기
  [pwr   u16]  SRP 피크*100 (신뢰도)

키: 'c' = 로그/측정 초기화
"""

import struct
from collections import deque
from threading import Thread, Event

import numpy as np
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# =========================================================
# 설정 (펌웨어 tdoa.h 와 일치)
# =========================================================

SERIAL_PORT = "COM3"
BAUD_RATE = 2000000

GRID_HALF = 0.10        # 격자 ±10cm (tdoa.h GRID_N=41, STEP=0.5cm)
ROI_R = 0.075           # 15cm 관심원 (반지름 7.5cm)
VIEW = GRID_HALF + 0.012

# 측정/로그 (관측 위해 임계 낮게; idle rms 보고 올려라)
RMS_LOG = 2.0           # 이 RMS 이상이면 측정/로그에 포함
RMS_MAX = 40.0          # 색이 빨강으로 포화되는 RMS
SMOOTH_N = 15           # 최근 N 프레임 중앙값으로 정밀 측정 (~0.3s)
MAXLOG = 2000

MAGIC = 0x3CC3
PKT = struct.Struct("<HHhhHH")   # magic, seq, x_mm, y_mm, rms, pwr (12 byte)
PKT_LEN = PKT.size

# =========================================================
# 공유 상태
# =========================================================

stop_event = Event()
new_dets = deque()    # (x, y, rms) — reader -> GUI
cur = {"x": 0.0, "y": 0.0, "rms": 0, "pwr": 0, "n": 0, "on": False}

# =========================================================
# Serial reader
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
            if len(buf) < 2 * PKT_LEN:
                continue
            if buf[PKT_LEN] == 0xC3 and buf[PKT_LEN + 1] == 0x3C:
                synced = True
            else:
                del buf[:1]
                continue

        while len(buf) >= PKT_LEN:
            magic, seq, xmm, ymm, rms, pwr = PKT.unpack_from(buf, 0)
            if magic != MAGIC:
                synced = False
                del buf[:1]
                break
            del buf[:PKT_LEN]

            x, y = xmm / 1000.0, ymm / 1000.0
            cur.update(x=x, y=y, rms=int(rms), pwr=int(pwr))
            cur["n"] += 1
            on = rms >= RMS_LOG
            cur["on"] = on
            if on:
                new_dets.append((x, y, int(rms)))

    ser.close()
    print("Serial closed.")

# =========================================================
# GUI
# =========================================================

pg.setConfigOptions(antialias=True, background="k", foreground="w")
app = pg.mkQApp("Live TDOA — precise")
win = pg.GraphicsLayoutWidget(show=True, title="Live TDOA — precise measurement")
win.resize(780, 820)

plot = win.addPlot()
plot.setAspectLocked(True)
plot.setXRange(-VIEW, VIEW)
plot.setYRange(-VIEW, VIEW)
plot.showGrid(x=True, y=True, alpha=0.2)
plot.setLabel("bottom", "X [m]")
plot.setLabel("left", "Y [m]")

# 15cm 관심원 + 격자 외곽 + 중심 십자
th = np.linspace(0, 2 * np.pi, 120)
plot.addItem(pg.PlotDataItem(ROI_R * np.cos(th), ROI_R * np.sin(th),
                             pen=pg.mkPen((255, 255, 255, 150), width=1.5)))
plot.addItem(pg.PlotDataItem(
    [-GRID_HALF, GRID_HALF, GRID_HALF, -GRID_HALF, -GRID_HALF],
    [-GRID_HALF, -GRID_HALF, GRID_HALF, GRID_HALF, -GRID_HALF],
    pen=pg.mkPen((100, 100, 100, 120))))
plot.addItem(pg.PlotDataItem([-ROI_R, ROI_R], [0, 0], pen=pg.mkPen((255, 255, 255, 35))))
plot.addItem(pg.PlotDataItem([0, 0], [-ROI_R, ROI_R], pen=pg.mkPen((255, 255, 255, 35))))

# 세기 -> 색 LUT
try:
    _cmap = pg.colormap.get("turbo")
except Exception:
    _cmap = pg.colormap.get("inferno")
LUT = _cmap.getLookupTable(0.0, 1.0, 256)

def brush_for(rms):
    t = (rms - RMS_LOG) / max(1e-6, (RMS_MAX - RMS_LOG))
    i = int(max(0, min(255, t * 255)))
    return pg.mkBrush(int(LUT[i][0]), int(LUT[i][1]), int(LUT[i][2]), 200)

# 누적 로그 + 현재(순간) 점 + 정밀 측정(중앙값) 마커
log_scatter = pg.ScatterPlotItem(size=8, pen=None)
plot.addItem(log_scatter)
xs, ys, brushes = [], [], []

cur_dot = pg.ScatterPlotItem(size=10, pen=None, brush=pg.mkBrush(255, 255, 255, 130))
plot.addItem(cur_dot)
meas_marker = pg.ScatterPlotItem(size=28, pen=pg.mkPen((0, 255, 0), width=3),
                                 brush=pg.mkBrush(0, 255, 0, 0), symbol="+")
plot.addItem(meas_marker)

readout = pg.TextItem("", color=(0, 255, 0), anchor=(0, 0))
readout.setPos(-VIEW * 0.98, VIEW * 0.96)
plot.addItem(readout)

recent = deque(maxlen=SMOOTH_N)   # 최근 (x,y) — 중앙값 측정용


def clear_all():
    xs.clear(); ys.clear(); brushes.clear()
    recent.clear()
    log_scatter.setData([], [])


try:
    _ShortcutCls = getattr(QtWidgets, "QShortcut", None) or pg.QtGui.QShortcut
    _sc = _ShortcutCls(pg.QtGui.QKeySequence("c"), win)
    _sc.activated.connect(clear_all)
except Exception as _e:
    print("clear shortcut unavailable:", _e)


def update():
    if stop_event.is_set() or not win.isVisible():
        return

    added = False
    while new_dets:
        try:
            x, y, rms = new_dets.popleft()
        except IndexError:
            break
        xs.append(x); ys.append(y); brushes.append(brush_for(rms))
        recent.append((x, y))
        added = True
    over = len(xs) - MAXLOG
    if over > 0:
        del xs[:over]; del ys[:over]; del brushes[:over]
        added = True
    if added:
        log_scatter.setData(xs, ys, brush=brushes, size=8, pen=None)

    # 현재(순간) 추정 — 항상 표시(켜져 있으면)
    if cur["on"]:
        cur_dot.setData([cur["x"]], [cur["y"]])
    else:
        cur_dot.setData([], [])

    # 정밀 측정: 최근 프레임 중앙값 ± std
    if len(recent) >= 3:
        arr = np.array(recent)
        mx, my = float(np.median(arr[:, 0])), float(np.median(arr[:, 1]))
        std_mm = float(np.hypot(np.std(arr[:, 0]), np.std(arr[:, 1]))) * 1000.0
        meas_marker.setData([mx], [my])
        in_roi = "IN" if (mx * mx + my * my) <= ROI_R * ROI_R else "out"
        readout.setText(
            f"측정: ({mx*100:+.2f}, {my*100:+.2f}) cm  ±{std_mm:.1f}mm  [{in_roi} 15cm]\n"
            f"rms {cur['rms']}  pwr {cur['pwr']/100:.2f}  n={len(recent)}  log={len(xs)}")
    else:
        meas_marker.setData([], [])
        readout.setText(
            f"측정 대기...  rms {cur['rms']}  pwr {cur['pwr']/100:.2f}  "
            f"(RMS_LOG={RMS_LOG:.0f} 미만이면 안 잡힘)")

    win.setWindowTitle(
        f"Live TDOA precise | rms {cur['rms']} | pwr {cur['pwr']/100:.2f} | "
        f"pkts {cur['n']} | log {len(xs)} | 'c'=clear")


# =========================================================
# 실행
# =========================================================

reader_thread = Thread(target=serial_reader, daemon=True)
reader_thread.start()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(30)


def on_quit():
    stop_event.set()
    reader_thread.join(timeout=1.0)


app.aboutToQuit.connect(on_quit)

print(f"Live TDOA precise. grid ±{GRID_HALF*100:.0f}cm, ROI 15cm. "
      f"RMS_LOG={RMS_LOG}. 현재 추정/측정은 항상 표시(진단용). 'c'=clear")

if __name__ == "__main__":
    try:
        pg.exec()
    except KeyboardInterrupt:
        stop_event.set()
