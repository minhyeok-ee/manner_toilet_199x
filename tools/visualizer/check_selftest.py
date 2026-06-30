"""
부팅 시 출력되는 ASCII(배너 + CMSIS-DSP self-test)만 뽑아 보여준다.
바이너리 스트림은 무시.

Portenta는 네이티브 USB(CDC)라 RESET 때 USB가 잠깐 끊긴다 -> 핸들 무효화.
이 스크립트는 그 끊김을 견디고 자동 재연결한다.

사용법:
  1) 이 스크립트를 실행
  2) "보드 RESET 누르세요" 뜨면 Portenta RESET 버튼을 한 번 누른다
     (USB가 끊겼다 다시 떠도 스크립트가 알아서 재연결해 부팅 출력을 잡는다)
"""

import time
import serial
from serial import SerialException

PORT = "COM3"
BAUD = 2000000
TOTAL_SECONDS = 20.0

KEYWORDS = ("CMSIS-DSP", "TDOA", "selftest", "Portenta", "Expected",
            "MODE", "Frame", "SAI", "ERROR", "streaming")


def try_open():
    try:
        return serial.Serial(PORT, BAUD, timeout=0.2)
    except SerialException:
        return None


def main():
    print(f"준비됨 ({PORT}). 이제 보드 RESET 버튼을 누르세요.")
    print(f"최대 {TOTAL_SECONDS:.0f}s 대기 — RESET 때 USB가 잠깐 끊겨도 자동 재연결합니다.")

    buf = bytearray()
    ser = try_open()
    found = False
    t0 = time.time()

    while time.time() - t0 < TOTAL_SECONDS:
        if ser is None:
            ser = try_open()           # 포트 재등장 대기 후 재오픈
            if ser is None:
                time.sleep(0.3)
                continue
        try:
            chunk = ser.read(8192)
        except SerialException:
            # RESET 으로 USB 끊김 -> 닫고 재오픈 시도
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(0.3)
            continue

        if chunk:
            buf += chunk
            # 부팅 self-test 들이 다 출력된 시점(스트리밍 시작) 또는 PASS/FAIL 까지 수신
            if b"streaming started" in buf or b"PASS" in buf or b"FAIL" in buf:
                found = b"selftest" in buf
                break

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass

    text = buf.decode("ascii", "ignore")
    seen = set()
    print("---- 부팅 ASCII 라인 ----")
    for line in text.splitlines():
        s = line.strip()
        if s and any(k in s for k in KEYWORDS) and s not in seen:
            print("  ", s)
            seen.add(s)
    print("-------------------------")
    if found:
        print("OK: self-test 라인 수신됨. (peak_bin=10 이면 FFT 정상)")
    else:
        print("self-test 미수신 — RESET을 한 번 더 누르거나, 포트 번호/점유를 확인하세요.")


if __name__ == "__main__":
    main()
