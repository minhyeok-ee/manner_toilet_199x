/*
 * tdoa.c — GCC-PHAT + SRP-PHAT 음원 위치추정 (CMSIS-DSP arm_cfft_f32)
 * tools/visualizer/tdoa_localize.py 의 검증 알고리즘을 그대로 포팅.
 *
 * (x,y) 결과는 FFT 스케일에 불변(PHAT가 크기를 1로 정규화 + argmax는 스케일 무관)
 * 이라 cfft/rfft, inverse 스케일 차이와 무관하게 Python 과 일치한다.
 */
#include "tdoa.h"

#include <math.h>
#include <float.h>
#include <arm_math.h>
#include <arm_const_structs.h>

/* ---- 파라미터 (tdoa_localize.py 와 일치) ---- */
#define NFFT       4096
#define FS         48000.0f
#define C_SOUND    343.0f
#define MAX_LAG    64                 /* >= 최대 baseline tau(~56샘플) */
#define NLAG       (2 * MAX_LAG + 1)
#define FMIN       200.0f
#define FMAX       8000.0f

#define GRID_MIN   (-0.35f)
#define GRID_MAX   ( 0.35f)
#define GRID_STEP  ( 0.01f)
#define GRID_N     71                 /* round((0.35-(-0.35))/0.01)+1 */

/* 어레이: 대각선 40cm, 긴 축(y)=전후방. CH 순서 = 좌하,우하,우상,좌상 */
#define ARR_W_MM   233
#define ARR_H_MM   326
#define ARR_W      (ARR_W_MM / 1000.0f)
#define ARR_H      (ARR_H_MM / 1000.0f)

/* 컴파일 가드: MAX_LAG 가 최대 |tau|(= 대각선/c*fs)를 덮어야 함.
 * 정수(mm) 비교로 sqrt 없이 검사. 어레이를 키우면 여기서 잡힌다.
 *   MAX_LAG*343000/48000 = lag 한계의 경로차(mm), 대각선^2 = W^2+H^2 (mm^2) */
_Static_assert(
    (MAX_LAG * 343000 / 48000) * (MAX_LAG * 343000 / 48000)
        >= (ARR_W_MM * ARR_W_MM + ARR_H_MM * ARR_H_MM),
    "MAX_LAG too small for array baseline; increase MAX_LAG (and NLAG)");

static const float MIC[TDOA_CH][2] = {
    { -ARR_W / 2, -ARR_H / 2 },   /* CH1 */
    {  ARR_W / 2, -ARR_H / 2 },   /* CH2 */
    {  ARR_W / 2,  ARR_H / 2 },   /* CH3 */
    { -ARR_W / 2,  ARR_H / 2 },   /* CH4 */
};
static const int PAIR[6][2] = {
    {0,1},{0,2},{0,3},{1,2},{1,3},{2,3}
};
#define NPAIR 6

/* ---- 작업 버퍼 (정적) ---- */
static float chSpec[TDOA_CH][2 * NFFT];   /* 채널별 복소 스펙트럼 (인터리브) */
static float Rwork[2 * NFFT];             /* 쌍별 교차스펙트럼/역FFT 작업 */
static float ccLag[NPAIR][NLAG];          /* 쌍별 상관(지연 -MAX_LAG..+MAX_LAG) */
static float hann[TDOA_WINDOW];
static uint8_t bandKeep[NFFT];

static int s_init = 0;

void tdoa_init(void)
{
    int n, k;
    for (n = 0; n < TDOA_WINDOW; n++) {
        hann[n] = 0.5f - 0.5f * cosf(2.0f * PI * (float)n / (float)(TDOA_WINDOW - 1));
    }
    for (k = 0; k < NFFT; k++) {
        float f = (k <= NFFT / 2) ? ((float)k * FS / NFFT)
                                  : ((float)(k - NFFT) * FS / NFFT);
        float af = fabsf(f);
        bandKeep[k] = (af >= FMIN && af <= FMAX) ? 1u : 0u;
    }
    s_init = 1;
}

/* np.interp 와 동일: ccLag[p] 를 분수 지연 tau(샘플)에서 선형보간(경계 클램프) */
static inline float interp_lag(const float *cc, float tau)
{
    float pos = tau + (float)MAX_LAG;          /* index axis: 0..2*MAX_LAG */
    if (pos <= 0.0f)            return cc[0];
    if (pos >= (float)(NLAG - 1)) return cc[NLAG - 1];
    int i0 = (int)pos;
    float frac = pos - (float)i0;
    return cc[i0] * (1.0f - frac) + cc[i0 + 1] * frac;
}

tdoa_result_t tdoa_localize(const float *win)
{
    tdoa_result_t res = { 0.0f, 0.0f, -FLT_MAX };
    int ch, k, p, a;

    if (!s_init) tdoa_init();

    /* 1) 채널별 전방 FFT: DC 제거 + 해닝 + zero-pad -> arm_cfft_f32 */
    for (ch = 0; ch < TDOA_CH; ch++) {
        float mean = 0.0f;
        for (k = 0; k < TDOA_WINDOW; k++) mean += win[k * TDOA_CH + ch];
        mean /= (float)TDOA_WINDOW;

        float *buf = chSpec[ch];
        for (k = 0; k < TDOA_WINDOW; k++) {
            buf[2 * k]     = (win[k * TDOA_CH + ch] - mean) * hann[k];
            buf[2 * k + 1] = 0.0f;
        }
        for (k = TDOA_WINDOW; k < NFFT; k++) {
            buf[2 * k]     = 0.0f;
            buf[2 * k + 1] = 0.0f;
        }
        arm_cfft_f32(&arm_cfft_sR_f32_len4096, buf, 0, 1);   /* forward */
    }

    /* 2) 쌍별 GCC-PHAT -> ccLag */
    for (p = 0; p < NPAIR; p++) {
        const float *Xi = chSpec[PAIR[p][0]];
        const float *Xj = chSpec[PAIR[p][1]];
        for (k = 0; k < NFFT; k++) {
            if (bandKeep[k]) {
                float ar = Xi[2 * k],     ai = Xi[2 * k + 1];
                float br = Xj[2 * k],     bi = Xj[2 * k + 1];
                /* Xi * conj(Xj) */
                float rr = ar * br + ai * bi;
                float ri = ai * br - ar * bi;
                float mag = sqrtf(rr * rr + ri * ri) + 1e-9f;   /* PHAT */
                Rwork[2 * k]     = rr / mag;
                Rwork[2 * k + 1] = ri / mag;
            } else {
                Rwork[2 * k]     = 0.0f;
                Rwork[2 * k + 1] = 0.0f;
            }
        }
        arm_cfft_f32(&arm_cfft_sR_f32_len4096, Rwork, 1, 1);   /* inverse */

        for (a = 0; a < NLAG; a++) {
            int lag = a - MAX_LAG;
            int idx = (lag >= 0) ? lag : (NFFT + lag);
            ccLag[p][a] = Rwork[2 * idx];                      /* 실수부 */
        }
    }

    /* 3) SRP-PHAT 그리드 탐색 (on-the-fly tau) */
    {
        int ix, iy;
        for (iy = 0; iy < GRID_N; iy++) {
            float y = GRID_MIN + (float)iy * GRID_STEP;
            for (ix = 0; ix < GRID_N; ix++) {
                float x = GRID_MIN + (float)ix * GRID_STEP;

                float dist[TDOA_CH];
                for (ch = 0; ch < TDOA_CH; ch++) {
                    float dx = x - MIC[ch][0];
                    float dy = y - MIC[ch][1];
                    dist[ch] = sqrtf(dx * dx + dy * dy);
                }
                float srp = 0.0f;
                for (p = 0; p < NPAIR; p++) {
                    float tau = (dist[PAIR[p][0]] - dist[PAIR[p][1]]) / C_SOUND * FS;
                    srp += interp_lag(ccLag[p], tau);
                }
                if (srp > res.power) {
                    res.power = srp;
                    res.x = x;
                    res.y = y;
                }
            }
        }
    }
    return res;
}
