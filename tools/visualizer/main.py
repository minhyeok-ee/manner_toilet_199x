import serial
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# =========================
# 설정
# =========================

SERIAL_PORT = "/dev/cu.usbmodem1101"  # macOS
# SERIAL_PORT = "COM7"                # Windows

BAUD_RATE = 2000000

# Arduino에서 RAW를 줄여서 보내기 때문에 실제 표시용 샘플레이트
# 처음에는 정확하지 않아도 됨.
# 그래프 주파수 축이 이상하면 이 값을 조정하면 됨.
SAMPLE_RATE = 12000

FFT_SIZE = 256
HOP_SIZE = 64

MAX_FREQ = 3000

HISTORY_FRAMES = 120

CHANNEL_COUNT = 4

# =========================
# 버퍼
# =========================

sample_buffers = [
    deque(maxlen=FFT_SIZE * 4) for _ in range(CHANNEL_COUNT)
]

spectrograms = [
    deque(maxlen=HISTORY_FRAMES) for _ in range(CHANNEL_COUNT)
]

freqs = np.fft.rfftfreq(FFT_SIZE, d=1.0 / SAMPLE_RATE)
freq_mask = freqs <= MAX_FREQ
freqs_view = freqs[freq_mask]

# =========================
# Serial 연결
# =========================

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)

# =========================
# Plot 설정
# =========================

plt.ion()

fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)

images = []

for ch in range(CHANNEL_COUNT):
    empty_img = np.zeros((len(freqs_view), HISTORY_FRAMES))

    img = axes[ch].imshow(
        empty_img,
        aspect="auto",
        origin="lower",
        extent=[0, HISTORY_FRAMES, freqs_view[0], freqs_view[-1]],
    )

    axes[ch].set_ylabel(f"CH{ch + 1}\nHz")
    axes[ch].set_ylim(0, MAX_FREQ)

    images.append(img)

axes[-1].set_xlabel("Time frame")
fig.suptitle("PCM1840 4CH Frequency Domain Spectrogram")

plt.tight_layout()

# =========================
# FFT 함수
# =========================

def calculate_fft_db(samples):
    samples = np.array(samples, dtype=np.float32)

    if len(samples) < FFT_SIZE:
        return None

    samples = samples[-FFT_SIZE:]

    # DC 제거
    samples = samples - np.mean(samples)

    # window 적용
    window = np.hanning(FFT_SIZE)
    samples = samples * window

    spectrum = np.fft.rfft(samples)
    magnitude = np.abs(spectrum)

    # dB 변환
    magnitude_db = 20 * np.log10(magnitude + 1e-6)

    return magnitude_db[freq_mask]


def update_plot():
    for ch in range(CHANNEL_COUNT):
        if len(spectrograms[ch]) == 0:
            continue

        spec = np.array(spectrograms[ch]).T

        images[ch].set_data(spec)
        images[ch].set_extent([0, spec.shape[1], freqs_view[0], freqs_view[-1]])

        vmin = np.percentile(spec, 5)
        vmax = np.percentile(spec, 95)

        if vmax > vmin:
            images[ch].set_clim(vmin, vmax)

    plt.pause(0.001)


# =========================
# Main loop
# =========================

print("Reading serial...")
print("Close plot window or press Ctrl+C to stop.")

sample_count = 0

try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()

        if not line.startswith("RAW,"):
            continue

        parts = line.split(",")

        if len(parts) < 6:
            continue

        try:
            values = [
                int(parts[2]),
                int(parts[3]),
                int(parts[4]),
                int(parts[5]),
            ]
        except ValueError:
            continue

        for ch in range(CHANNEL_COUNT):
            sample_buffers[ch].append(values[ch])

        sample_count += 1

        if sample_count % HOP_SIZE == 0:
            for ch in range(CHANNEL_COUNT):
                fft_db = calculate_fft_db(sample_buffers[ch])

                if fft_db is not None:
                    spectrograms[ch].append(fft_db)

            update_plot()

except KeyboardInterrupt:
    print("Stopped.")

finally:
    ser.close()