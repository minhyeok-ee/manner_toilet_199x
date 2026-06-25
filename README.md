# Toilet Noise Cancellation

Arduino Portenta H7와 PCM1840 4채널 ADC를 사용하여 변기 소음을 수집하고, 이후 위치 추정, 소리 분류, 능동 소음 제어(ANC)를 구현하기 위한 프로젝트입니다.

현재 단계에서는 전체 ANC 구현 전에 **PCM1840에서 4채널 오디오 데이터를 정상적으로 수집하는 것**과 **Python으로 실시간 시각화하는 것**을 목표로 합니다.

---

## 1. Project Goal

최종 목표는 다음과 같습니다.

```text
마이크 4개로 변기 소음 수집
→ 소리 방향 / 위치 추정
→ 변기 소리 여부 판단
→ 역위상 소리 생성
→ 스피커 출력
→ 에러 마이크로 상쇄 결과 확인
→ 적응형 필터로 보정
```

현재 구현 단계는 다음입니다.

```text
Portenta H7
+ PCM1840 ADC
+ SAI TDM 4채널 입력
+ Serial 데이터 전송
+ Python 실시간 주파수 도메인 시각화
```

---

## 2. Hardware

### Main Components

| Component           | Description                                |
| ------------------- | ------------------------------------------ |
| Arduino Portenta H7 | Main MCU board                             |
| PCM1840             | 4-channel audio ADC                        |
| Microphones         | 4 input microphones                        |
| PC                  | Serial monitoring and Python visualization |

---

## 3. Audio Input Architecture

```text
Microphone 1 ─┐
Microphone 2 ─┤
Microphone 3 ─┤
Microphone 4 ─┘
        ↓
     PCM1840
        ↓  TDM 4CH
   Portenta H7 SAI
        ↓
      Serial
        ↓
 Python Visualizer
```

---

## 4. PCM1840 Mode Setting

PCM1840은 하드웨어 핀 설정으로 동작 모드를 결정합니다.

현재 프로젝트에서는 다음 설정을 사용합니다.

| PCM1840 Pin | Connection | Meaning             |
| ----------- | ---------- | ------------------- |
| MSZ         | GND        | Slave mode          |
| FMT0        | GND        | 4-channel TDM       |
| FMT1        | GND        | 4-channel TDM       |
| MD0         | GND        | Linear phase filter |
| MD1         | GND        | DRE disabled        |
| SHDNZ       | 3.3V       | ADC enabled         |
| AVDD        | 3.3V       | Analog power        |
| IOVDD       | 3.3V       | Digital I/O power   |
| GND         | GND        | Common ground       |

---

## 5. Portenta H7 ↔ PCM1840 Connection

현재는 Arduino 기본 `I2S` 라이브러리 대신 STM32 HAL의 `SAI`를 사용합니다.

| PCM1840      | Portenta H7  | Description       |
| ------------ | ------------ | ----------------- |
| BCLK         | J2-49 SAI CK | Bit clock         |
| FSYNC        | J2-51 SAI FS | Frame sync        |
| SDOUT        | J2-53 SAI D0 | Serial audio data |
| GND          | GND          | Common ground     |
| AVDD / IOVDD | 3.3V         | Power             |

---

## 6. Expected Clock

현재 테스트 기준:

| Item           | Value     |
| -------------- | --------- |
| Sample rate    | 48 kHz    |
| Channel count  | 4         |
| Word size      | 32-bit    |
| Frame size     | 128-bit   |
| Expected BCLK  | 6.144 MHz |
| Expected FSYNC | 48 kHz    |

Calculation:

```text
BCLK = sample rate × channel count × word size
BCLK = 48000 × 4 × 32
BCLK = 6.144 MHz
```

---

## 7. Project Structure

현재 권장 구조는 다음과 같습니다.

```text
toilet-noise-cancellation/
├── platformio.ini
├── src/
│   └── main.cpp
├── include/
├── lib/
├── test/
├── tools/
│   └── spectrogram_4ch.py
├── docs/
│   └── hardware.md
└── README.md
```

나중에 프로젝트가 커지면 다음 구조로 확장할 수 있습니다.

```text
toilet-noise-cancellation/
├── firmware/
│   ├── src/
│   │   ├── main.cpp
│   │   ├── audio_input.cpp
│   │   ├── audio_output.cpp
│   │   ├── tdoa.cpp
│   │   ├── classifier.cpp
│   │   └── anc.cpp
│   └── include/
│       ├── audio_input.h
│       ├── audio_output.h
│       ├── tdoa.h
│       ├── classifier.h
│       └── anc.h
│
├── visualizer/
│   ├── serial_plot.py
│   ├── fft_viewer.py
│   └── spectrogram_viewer.py
│
├── dataset/
├── training/
├── experiments/
└── docs/
```

---

## 8. PlatformIO Setup

`platformio.ini`

```ini
[env:portenta_h7_m7]
platform = ststm32
board = portenta_h7_m7
framework = arduino

upload_protocol = dfu
monitor_speed = 2000000

build_flags =
    -DHAL_SAI_MODULE_ENABLED
```

---

## 9. Firmware

현재 펌웨어의 역할은 다음과 같습니다.

1. STM32 HAL SAI 초기화
2. SAI2 Block A를 Master RX로 설정
3. PCM1840에서 4채널 TDM 데이터 수신
4. 수신 데이터를 Serial로 출력
5. Python에서 읽을 수 있도록 CSV 형태로 전송

Serial 출력 형식:

```text
RAW,seq,ch1,ch2,ch3,ch4
```

예시:

```text
RAW,0,12,-3,5,8
RAW,1,14,-2,6,9
RAW,2,15,-1,7,11
```

---

## 10. Python Visualizer

현재 Python 시각화 코드는 Serial로 들어오는 4채널 데이터를 읽고, 각 채널을 주파수 도메인으로 변환하여 스펙트로그램으로 표시합니다.

그래프 의미:

| Axis / Color     | Meaning                          |
| ---------------- | -------------------------------- |
| x-axis           | Time frame                       |
| y-axis           | Frequency in Hz                  |
| Color brightness | Magnitude of frequency component |

---

## 11. Run Visualizer

macOS에서 Serial 포트 확인:

```bash
ls /dev/cu.usbmodem*
```

Windows에서는 장치 관리자에서 COM 포트를 확인합니다.

Python 코드에서 포트를 수정합니다.

```python
SERIAL_PORT = "/dev/cu.usbmodemXXXX"
```

또는 Windows:

```python
SERIAL_PORT = "COM7"
```

실행:

```bash
python tools/spectrogram_4ch.py
```

---

## 12. Test Procedure

1. PCM1840 전원과 GND 연결 확인
2. PCM1840 모드 설정 핀 확인
3. Portenta H7와 PCM1840의 SAI 핀 연결 확인
4. PlatformIO로 펌웨어 업로드
5. Serial Monitor에서 RAW 데이터 확인
6. Python visualizer 실행
7. 마이크 1~4번을 하나씩 두드려서 채널 반응 확인
8. 스펙트로그램에서 주파수 성분 변화 확인

---

## 13. Current Status

현재까지 완료한 내용:

* Portenta H7 + PlatformIO 환경 기준 설정
* PCM1840 4채널 TDM 입력 구조 정리
* Arduino 기본 I2S 대신 STM32 HAL SAI 방식으로 전환
* SAI Master RX 테스트 코드 작성
* Serial RAW 데이터 출력 형식 정의
* Python 실시간 스펙트로그램 시각화 코드 작성
* GitHub 업로드용 `.gitignore` 정리

---

## 14. Next Steps

다음 단계는 다음 순서로 진행할 예정입니다.

1. SAI 입력 안정화
2. DMA circular buffer 적용
3. 4채널 데이터 분리 검증
4. Python waveform / RMS / FFT / spectrogram 시각화 분리
5. CSV 또는 WAV 저장 기능 추가
6. 마이크 캘리브레이션
7. TDOA 기반 방향 추정
8. 변기 소리 데이터셋 구축
9. Target sound 분류
10. 단순 ANC 실험
11. FxLMS 기반 ANC 구현

---

## 15. Notes

현재 코드는 실시간 ANC용 최종 구조가 아니라 **하드웨어 입력 검증용 테스트 코드**입니다.

최종 구현에서는 polling 방식 대신 다음 구조가 필요합니다.

```text
SAI RX
↓
DMA circular buffer
↓
4-channel deinterleaving
↓
DSP processing
↓
classification / TDOA / ANC
↓
audio output
```
