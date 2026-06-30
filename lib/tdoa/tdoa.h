/*
 * tdoa.h — 4-mic 근거리 TDOA 음원 위치추정 (GCC-PHAT + SRP-PHAT)
 *
 * tools/visualizer/tdoa_localize.py 의 검증된 알고리즘을 C로 포팅.
 * FFT 는 CMSIS-DSP(arm_cfft_f32) 사용.
 *
 * 좌표계: 어레이 중심 원점, 단위 meter, 모든 마이크 z=0 평면(2D).
 */
#ifndef TDOA_H
#define TDOA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TDOA_CH      4
#define TDOA_WINDOW  2048      /* 위치 계산 윈도우 (~42.7ms @48k) */

typedef struct {
    float x;      /* meter */
    float y;      /* meter */
    float power;  /* SRP 피크값 (신뢰도 지표) */
} tdoa_result_t;

/* 그리드/테이블 1회 초기화 (해닝 윈도우, FFT 인스턴스). 시작 시 1회 호출. */
void tdoa_init(void);

/*
 * 위치 추정.
 *   win : 길이 TDOA_WINDOW*TDOA_CH, 프레임-major 인터리브
 *         win[n*TDOA_CH + ch] = 샘플 n, 채널 ch
 * 반환: 추정 (x,y) [meter] + SRP 피크값.
 */
tdoa_result_t tdoa_localize(const float *win);

#ifdef __cplusplus
}
#endif

#endif /* TDOA_H */
