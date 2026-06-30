"""
tdoa.c 검증용 테스트벡터 생성기.

알려진 위치의 합성 음원 -> 4ch 윈도우 생성 -> 검증된 SRP-PHAT 로 기대 (x,y) 계산
-> src/tdoa_testvec.h 로 출력 (입력 윈도우 + 기대값).

파라미터는 tdoa.c 와 반드시 일치해야 함.
펌웨어 self-test 가 tdoa_localize(tdoa_testvec) 결과를 TV_EXP_X/Y 와 대조한다.
"""

import numpy as np

# ---- tdoa.c 와 일치하는 파라미터 ----
CH = 4
WINDOW = 2048
NFFT = 4096
FS = 48000.0
C = 343.0
MAX_LAG = 64
FMIN, FMAX = 200.0, 8000.0
GRID_MIN, GRID_MAX, GRID_STEP = -0.35, 0.35, 0.01
ARR_W, ARR_H = 0.233, 0.326

MIC = np.array([
    [-ARR_W/2, -ARR_H/2],
    [ ARR_W/2, -ARR_H/2],
    [ ARR_W/2,  ARR_H/2],
    [-ARR_W/2,  ARR_H/2],
], dtype=np.float64)
PAIRS = [(0,1),(0,2),(0,3),(1,2),(1,3),(2,3)]

TRUE_SRC = np.array([0.05, -0.08])   # 알려진 음원 위치 (그리드 위)
AMP = 8000.0

# ---- 그리드 / lag / 밴드마스크 (cfft 와 동치인 rfft 사용) ----
LAGS = np.arange(-MAX_LAG, MAX_LAG + 1)
gx = np.arange(GRID_MIN, GRID_MAX + 1e-9, GRID_STEP)
gy = np.arange(GRID_MIN, GRID_MAX + 1e-9, GRID_STEP)
GX, GY = np.meshgrid(gx, gy, indexing="xy")
PTS = np.stack([GX.ravel(), GY.ravel()], axis=1)
dgrid = np.linalg.norm(PTS[:, None, :] - MIC[None, :, :], axis=2)
TAU = np.empty((len(PAIRS), PTS.shape[0]))
for k, (i, j) in enumerate(PAIRS):
    TAU[k] = (dgrid[:, i] - dgrid[:, j]) / C * FS

rfreq = np.fft.rfftfreq(NFFT, d=1.0 / FS)
BAND = ((rfreq >= FMIN) & (rfreq <= FMAX)).astype(np.float64)
hann = np.hanning(WINDOW)


def synth_window(src, seed=7):
    """알려진 위치 음원 -> 분수지연 4ch 윈도우 (광대역 <=8kHz)."""
    L = NFFT
    rng = np.random.default_rng(seed)
    s = rng.standard_normal(L) * np.hanning(L)
    fn = np.fft.rfftfreq(L)                 # normalized
    S = np.fft.rfft(s)
    S[fn > FMAX / FS] = 0.0
    dist = np.linalg.norm(src - MIC, axis=1)
    tau = (dist - dist.min()) / C * FS      # samples
    out = np.empty((L, CH))
    for c in range(CH):
        out[:, c] = np.fft.irfft(S * np.exp(-1j * 2 * np.pi * fn * tau[c]), n=L)
    mid = L // 2
    w = out[mid - WINDOW // 2: mid + WINDOW // 2, :] * AMP
    return w.astype(np.float32)


def srp_phat(win):
    x = win - win.mean(axis=0, keepdims=True)
    X = np.fft.rfft(x * hann[:, None], n=NFFT, axis=0)
    srp = np.zeros(PTS.shape[0])
    for k, (i, j) in enumerate(PAIRS):
        R = X[:, i] * np.conj(X[:, j])
        R /= np.abs(R) + 1e-9
        R *= BAND
        cc = np.fft.irfft(R, n=NFFT)
        cc = np.concatenate((cc[-MAX_LAG:], cc[:MAX_LAG + 1]))
        srp += np.interp(TAU[k], LAGS, cc)
    b = int(np.argmax(srp))
    return PTS[b], srp[b]


def main():
    win = synth_window(TRUE_SRC)
    est, power = srp_phat(win)
    print(f"true=({TRUE_SRC[0]*100:+.1f},{TRUE_SRC[1]*100:+.1f})cm  "
          f"expected=({est[0]*100:+.1f},{est[1]*100:+.1f})cm  power={power:.3f}")

    flat = win.reshape(-1)   # frame-major: [n*CH + ch] (numpy row-major)
    lines = []
    for i in range(0, flat.size, 8):
        # 지수 표기로 항상 유효한 float 리터럴 보장 (정수형 'NNNf' 회피)
        chunk = ", ".join(f"{v:.8e}f" for v in flat[i:i + 8])
        lines.append("    " + chunk + ",")
    body = "\n".join(lines)

    header = f"""/* 자동 생성: tools/visualizer/gen_tdoa_testvec.py — 수정 금지 */
#ifndef TDOA_TESTVEC_H
#define TDOA_TESTVEC_H

#define TV_TRUE_X  ({TRUE_SRC[0]:.4f}f)
#define TV_TRUE_Y  ({TRUE_SRC[1]:.4f}f)
#define TV_EXP_X   ({est[0]:.4f}f)   /* SRP 기대 결과 = C 가 내야 할 값 */
#define TV_EXP_Y   ({est[1]:.4f}f)

/* win[n*4 + ch], n=0..2047, ch=0..3 */
static const float tdoa_testvec[{flat.size}] = {{
{body}
}};

#endif
"""
    out_path = "src/tdoa_testvec.h"
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(header)
    print(f"wrote {out_path} ({flat.size} floats)")


if __name__ == "__main__":
    main()
