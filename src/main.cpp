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

// 수신 버퍼
static uint32_t rxBuffer[WORDS_PER_BLOCK];

static uint32_t blockSeq = 0;
static uint32_t rawSeq = 0;

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
    Serial.println("STAT format:");
    Serial.println("STAT,seq,rms1,rms2,rms3,rms4,peak1,peak2,peak3,peak4,mean1,mean2,mean3,mean4");
    Serial.println("RAW format:");
    Serial.println("RAW,seq,ch1,ch2,ch3,ch4");

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
}

void loop() {
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