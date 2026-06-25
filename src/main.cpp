#include <Arduino.h>
#include <math.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_sai.h"

// ======================================================
// Portenta H7 + PCM1840 SAI TDM 4CH Test
//
// Portenta H7 : SAI Master RX
// PCM1840     : TDM 4CH Slave
// Sample rate : 48 kHz
// Word size   : 32-bit
//
// Expected:
// FSYNC = 48 kHz
// BCLK  = 48,000 * 4ch * 32bit = 6.144 MHz
// ======================================================

static SAI_HandleTypeDef hsai2a;

// -------------------- Test settings --------------------
static constexpr uint32_t SAMPLE_RATE = 48000;
static constexpr uint32_t CHANNEL_COUNT = 4;
static constexpr uint32_t BITS_PER_SAMPLE = 32;

// 한 번에 읽을 프레임 수
static constexpr uint16_t FRAMES_PER_BLOCK = 64;

// SAI에서 읽는 32-bit word 개수
static constexpr uint16_t WORDS_PER_BLOCK = FRAMES_PER_BLOCK * CHANNEL_COUNT;

// Serial 출력 속도 조절
static constexpr uint32_t PRINT_STATS_EVERY_N_BLOCKS = 20;

// RAW 출력은 파형 확인용.
// 너무 많이 출력하면 Serial이 밀릴 수 있으므로 적게 출력.
static constexpr bool PRINT_RAW_SAMPLES = true;
static constexpr uint32_t PRINT_RAW_EVERY_N_BLOCKS = 50;
static constexpr uint16_t RAW_FRAMES_TO_PRINT = 8;

// ======================================================
// 바이너리 스트리밍 모드 (FFT 시각화용)
//
// true  : 48kHz를 RAW_DECIMATION 배수로 연속 데시메이션하여
//         12바이트 바이너리 프레임으로 끊김 없이 스트리밍.
//         (STAT / RAW ASCII 출력은 비활성화 -> 스트림이 깨끗해짐)
// false : 기존 STAT / RAW ASCII 디버그 출력 동작.
//
// 프레임 포맷 (Little-Endian, 12 byte):
//   [0xA5][0x5A]  magic (2)
//   [seq  u16]    프레임 시퀀스, 0~65535 wrap (2)
//   [ch0  i16]
//   [ch1  i16]
//   [ch2  i16]
//   [ch3  i16]    각 채널 샘플 (8)
//
// RAW_DECIMATION = 6  -> 48000 / 6  = 8000 Hz  (관측 가능 ~4000 Hz)
// RAW_DECIMATION = 3  -> 48000 / 3  = 16000 Hz (관측 가능 ~8000 Hz)
// RAW_DECIMATION = 12 -> 48000 / 12 = 4000 Hz  (관측 가능 ~2000 Hz)
//
// 주의: 단순 평균(boxcar) 데시메이션이라 안티앨리어싱은 약함.
//       정밀 측정이 필요하면 별도 저역통과 필터를 둘 것.
// ======================================================
static constexpr bool STREAM_BINARY = true;
static constexpr uint16_t RAW_DECIMATION = 3;
static constexpr uint8_t FRAME_BYTES = 4 + CHANNEL_COUNT * 2; // magic+seq + 4*i16 = 12

// 수신 버퍼 (레거시 블로킹 모드용)
static uint32_t rxBuffer[WORDS_PER_BLOCK];

static uint32_t blockSeq = 0;
static uint32_t rawSeq = 0;

// ------------------------------------------------------
// DMA 순환버퍼 (STREAM_BINARY 모드)
//
// 핑퐁: DMA가 한 절반을 채우는 동안 CPU는 반대 절반을 처리.
// IRQ를 쓰지 않고 메인루프에서 NDTR 카운터를 폴링해 완성 절반을 감지
// -> mbed 코어가 선점한 DMA IRQ 핸들러와 충돌하지 않음.
//
// 캐시: dmaBuf 는 캐시 가능한 AXI SRAM 에 잡히므로, 각 절반을 읽기 전
//       SCB_InvalidateDCache_by_Addr 로 무효화해야 DMA 최신값을 읽는다.
//       이를 위해 32바이트 정렬 + 절반 크기를 32바이트 배수로 둔다.
// ------------------------------------------------------
static constexpr uint16_t HALF_FRAMES = 256;                       // 절반당 프레임
static constexpr uint16_t HALF_WORDS = HALF_FRAMES * CHANNEL_COUNT; // 1024 word
static constexpr uint16_t DMA_TOTAL_WORDS = HALF_WORDS * 2;         // 2048 word

// HALF_WORDS*4 = 4096 byte 는 32 의 배수여야 캐시라인 단위로 정확히 무효화됨
static_assert((HALF_WORDS * sizeof(uint32_t)) % 32 == 0,
              "DMA half size must be a multiple of 32 bytes for cache ops");

static uint32_t dmaBuf[DMA_TOTAL_WORDS] __attribute__((aligned(32)));
static DMA_HandleTypeDef hdma_sai2_rx;

// ------------------------------------------------------
// Portenta H7 High Density Connector 기준
//
// J2-49 SAI CK  -> PI5 -> PCM1840 BCLK
// J2-51 SAI FS  -> PI7 -> PCM1840 FSYNC
// J2-53 SAI D0  -> PI6 -> PCM1840 SDOUT
//
// STM32H747 기준 SAI2 Block A 사용
// ------------------------------------------------------
extern "C" void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hsai->Instance == SAI2_Block_A) {
        __HAL_RCC_GPIOI_CLK_ENABLE();
        __HAL_RCC_SAI2_CLK_ENABLE();

        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;

        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

        // -------- SAI2_A RX DMA (순환, 폴링) --------
        // DMA1/DMA2 는 D2 도메인 마스터라 SAI2 와 AXI SRAM 모두 접근 가능.
        // NVIC IRQ 는 일부러 켜지 않음: 메인루프에서 NDTR 폴링으로 처리.
        __HAL_RCC_DMA1_CLK_ENABLE();

        hdma_sai2_rx.Instance = DMA1_Stream0;
        hdma_sai2_rx.Init.Request = DMA_REQUEST_SAI2_A;
        hdma_sai2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_sai2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_sai2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_sai2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_sai2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_sai2_rx.Init.Mode = DMA_CIRCULAR;
        hdma_sai2_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_sai2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

        HAL_DMA_Init(&hdma_sai2_rx);

        __HAL_LINKDMA(hsai, hdmarx, hdma_sai2_rx);
    }
}

static bool initSAI2_TDM_RX() {
    hsai2a.Instance = SAI2_Block_A;

    hsai2a.Init.Protocol = SAI_FREE_PROTOCOL;
    hsai2a.Init.AudioMode = SAI_MODEMASTER_RX;
    hsai2a.Init.DataSize = SAI_DATASIZE_32;
    hsai2a.Init.FirstBit = SAI_FIRSTBIT_MSB;

    // PCM1840 TDM 데이터는 BCLK 기준으로 출력됨.
    // 수신 측은 반대 엣지에서 샘플링하는 것이 안전함.
    hsai2a.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;

    hsai2a.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai2a.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
    hsai2a.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
    hsai2a.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
    hsai2a.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
    hsai2a.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai2a.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai2a.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai2a.Init.TriState = SAI_OUTPUT_NOTRELEASED;

    // TDM 4채널:
    // 4ch * 32bit = 128 BCLK per frame
    hsai2a.FrameInit.FrameLength = CHANNEL_COUNT * BITS_PER_SAMPLE; // 128
    hsai2a.FrameInit.ActiveFrameLength = 1;                         // 1 BCLK pulse
    hsai2a.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
    hsai2a.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
    hsai2a.FrameInit.FSOffset = SAI_FS_FIRSTBIT;

    hsai2a.SlotInit.FirstBitOffset = 0;
    hsai2a.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
    hsai2a.SlotInit.SlotNumber = CHANNEL_COUNT;
    hsai2a.SlotInit.SlotActive =
        SAI_SLOTACTIVE_0 |
        SAI_SLOTACTIVE_1 |
        SAI_SLOTACTIVE_2 |
        SAI_SLOTACTIVE_3;

    HAL_StatusTypeDef result = HAL_SAI_Init(&hsai2a);

    return result == HAL_OK;
}

static int32_t abs32(int32_t value) {
    return value < 0 ? -value : value;
}

// 32-bit PCM을 테스트용 16-bit 크기로 줄임.
// RMS/Peak 확인에는 이 정도면 충분함.
static int32_t toTestSample(uint32_t raw) {
    int32_t signed32 = static_cast<int32_t>(raw);
    return signed32 >> 16;
}

static void printStats() {
    int64_t sum[CHANNEL_COUNT] = {0};
    int64_t sumSq[CHANNEL_COUNT] = {0};
    int32_t peak[CHANNEL_COUNT] = {0};

    for (uint16_t frame = 0; frame < FRAMES_PER_BLOCK; frame++) {
        for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
            uint32_t raw = rxBuffer[frame * CHANNEL_COUNT + ch];
            int32_t sample = toTestSample(raw);

            sum[ch] += sample;
            sumSq[ch] += static_cast<int64_t>(sample) * sample;

            int32_t a = abs32(sample);
            if (a > peak[ch]) {
                peak[ch] = a;
            }
        }
    }

    int32_t mean[CHANNEL_COUNT];
    int32_t rms[CHANNEL_COUNT];

    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        mean[ch] = static_cast<int32_t>(sum[ch] / FRAMES_PER_BLOCK);
        rms[ch] = static_cast<int32_t>(
            sqrt(static_cast<double>(sumSq[ch]) / FRAMES_PER_BLOCK)
        );
    }

    Serial.print("STAT,");
    Serial.print(blockSeq);

    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        Serial.print(',');
        Serial.print(rms[ch]);
    }

    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        Serial.print(',');
        Serial.print(peak[ch]);
    }

    for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
        Serial.print(',');
        Serial.print(mean[ch]);
    }

    Serial.println();
}

// ------------------------------------------------------
// 바이너리 스트리밍: 블록을 연속 데시메이션하여 전송
// ------------------------------------------------------
static int32_t decimAcc[CHANNEL_COUNT] = {0};
static uint16_t decimCount = 0;
static uint16_t streamSeq = 0;

static inline void putLE16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

// 임의 버퍼(frames 프레임)를 연속 데시메이션하여 바이너리 프레임으로 송출.
// decimAcc / decimCount 가 static 이라 호출 간(=DMA 절반 간) 데시메이션이 이어짐.
static void streamBinaryFrames(const uint32_t *buf, uint16_t frames) {
    // 최대 frames/RAW_DECIMATION (+여유 2) 프레임 발생.
    static uint8_t txBuf[(HALF_FRAMES / RAW_DECIMATION + 2) * FRAME_BYTES];
    uint16_t len = 0;

    for (uint16_t frame = 0; frame < frames; frame++) {
        for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
            decimAcc[ch] += toTestSample(buf[frame * CHANNEL_COUNT + ch]);
        }

        if (++decimCount >= RAW_DECIMATION) {
            uint8_t *p = &txBuf[len];

            p[0] = 0xA5;
            p[1] = 0x5A;
            putLE16(&p[2], streamSeq++);

            for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
                int16_t s = static_cast<int16_t>(decimAcc[ch] / RAW_DECIMATION);
                putLE16(&p[4 + ch * 2], static_cast<uint16_t>(s));
                decimAcc[ch] = 0;
            }

            len += FRAME_BYTES;
            decimCount = 0;
        }
    }

    if (len > 0) {
        Serial.write(txBuf, len);
    }
}

// DMA 순환버퍼에서 완성된 절반을 폴링으로 감지 -> 캐시 무효화 -> 송출.
static void pollAndStreamDMA() {
    // lastDone: 마지막으로 처리한 절반(0=앞, 1=뒤). 시작값 1 -> 첫 처리는
    // DMA 가 앞 절반을 다 채우고 뒤로 넘어간 시점의 '앞 절반'(진짜 완성본)부터.
    static int lastDone = 1;

    uint16_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_sai2_rx);

    // DMA 기록 위치 = DMA_TOTAL_WORDS - ndtr.
    // ndtr > HALF_WORDS  -> DMA 가 '앞 절반'을 쓰는 중 -> '뒤 절반'이 완성됨.
    bool writingFirstHalf = (ndtr > HALF_WORDS);

    if (writingFirstHalf) {
        if (lastDone != 1) {
            uint32_t *half = &dmaBuf[HALF_WORDS];
            SCB_InvalidateDCache_by_Addr(half, HALF_WORDS * sizeof(uint32_t));
            streamBinaryFrames(half, HALF_FRAMES);
            lastDone = 1;

            blockSeq++;
            if (blockSeq % 50 == 0) {
                digitalWrite(LEDB, !digitalRead(LEDB));
            }
        }
    } else {
        if (lastDone != 0) {
            uint32_t *half = &dmaBuf[0];
            SCB_InvalidateDCache_by_Addr(half, HALF_WORDS * sizeof(uint32_t));
            streamBinaryFrames(half, HALF_FRAMES);
            lastDone = 0;

            blockSeq++;
            if (blockSeq % 50 == 0) {
                digitalWrite(LEDB, !digitalRead(LEDB));
            }
        }
    }
}

static void printRawSamples() {
    for (uint16_t frame = 0; frame < RAW_FRAMES_TO_PRINT; frame++) {
        Serial.print("RAW,");
        Serial.print(rawSeq++);

        for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
            uint32_t raw = rxBuffer[frame * CHANNEL_COUNT + ch];
            int32_t sample = toTestSample(raw);

            Serial.print(',');
            Serial.print(sample);
        }

        Serial.println();
    }
}

void setup() {
    Serial.begin(2000000);

    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < 3000) {
        delay(10);
    }

    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDB, HIGH);

    Serial.println("Portenta H7 + PCM1840 SAI TDM 4CH test start");
    Serial.println("Expected FSYNC: 48000 Hz");
    Serial.println("Expected BCLK : 6144000 Hz");

    if (STREAM_BINARY) {
        Serial.print("MODE: BINARY STREAM, decim=");
        Serial.print(RAW_DECIMATION);
        Serial.print(", out_rate=");
        Serial.print(SAMPLE_RATE / RAW_DECIMATION);
        Serial.println(" Hz");
        Serial.println("Frame: [A5 5A][seq u16][ch0..3 i16] LE, 12 bytes");
    } else {
        Serial.println("STAT format:");
        Serial.println("STAT,seq,rms1,rms2,rms3,rms4,peak1,peak2,peak3,peak4,mean1,mean2,mean3,mean4");
        Serial.println("RAW format:");
        Serial.println("RAW,seq,ch1,ch2,ch3,ch4");
    }

    if (!initSAI2_TDM_RX()) {
        Serial.println("ERROR: SAI2 init failed");

        while (true) {
            digitalWrite(LEDB, LOW);
            delay(100);
            digitalWrite(LEDB, HIGH);
            delay(100);
        }
    }

    Serial.println("SAI2 init OK");

    if (STREAM_BINARY) {
        // 순환 DMA 시작 (한 번만 호출, 이후 하드웨어가 계속 채움)
        HAL_StatusTypeDef dmaStatus = HAL_SAI_Receive_DMA(
            &hsai2a,
            reinterpret_cast<uint8_t *>(dmaBuf),
            DMA_TOTAL_WORDS
        );

        if (dmaStatus != HAL_OK) {
            Serial.print("ERROR: SAI DMA start failed, status=");
            Serial.println(static_cast<int>(dmaStatus));
        } else {
            Serial.println("SAI DMA streaming started");
        }
    }
}

void loop() {
    if (STREAM_BINARY) {
        // 순환 DMA + 캐시 무효화 기반 스트리밍 (블로킹 수신 없음).
        // 절반(256 frame @48k ≈ 5.3ms)보다 빠르게만 폴링하면 손실 없음.
        pollAndStreamDMA();
        return;
    }

    // ---- 레거시 ASCII 디버그 모드 (블로킹 폴링 수신) ----
    HAL_StatusTypeDef status = HAL_SAI_Receive(
        &hsai2a,
        reinterpret_cast<uint8_t *>(rxBuffer),
        WORDS_PER_BLOCK,
        1000
    );

    if (status == HAL_OK) {
        blockSeq++;

        // LED 토글: 데이터 수신 중인지 눈으로 확인
        if (blockSeq % 100 == 0) {
            digitalWrite(LEDB, !digitalRead(LEDB));
        }

        if (blockSeq % PRINT_STATS_EVERY_N_BLOCKS == 0) {
            printStats();
        }

        if (PRINT_RAW_SAMPLES && blockSeq % PRINT_RAW_EVERY_N_BLOCKS == 0) {
            printRawSamples();
        }
    } else {
        Serial.print("ERROR: HAL_SAI_Receive failed, status=");
        Serial.print(static_cast<int>(status));
        Serial.print(", hal_error=");
        Serial.println(HAL_SAI_GetError(&hsai2a));

        delay(500);
    }
}