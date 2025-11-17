#!/usr/bin/env python3
#하드웨어 PWM을 사용하였다. 그전에는 소프트웨어 PWM을 사용함

# ⬇️⬇️⬇️ [수정] RPi.GPIO 대신 pigpio 임포트 ⬇️⬇️⬇️
import pigpio
# ⬆️⬆️⬆️ [수정] 완료 ⬆️⬆️⬆️

import serial
import threading
import time
import atexit
import math

# --- 1. 핀 설정 (BCM 모드 기준) ---
L_FRONT_PWM_PIN = 12
L_FRONT_DIR_PIN = 23
L_REAR_PWM_PIN = 13
L_REAR_DIR_PIN  = 24
R_FRONT_PWM_PIN = 18
R_FRONT_DIR_PIN = 25
R_REAR_PWM_PIN  = 19
R_REAR_DIR_PIN  = 8
LEFT_SENSOR_PIN   = 17
CENTER_SENSOR_PIN = 27
RIGHT_SENSOR_PIN  = 22
LEFT_ENCODER_PULSE_PIN = 5
LEFT_ENCODER_DIR_PIN   = 6
RIGHT_ENCODER_PULSE_PIN = 26
RIGHT_ENCODER_DIR_PIN  = 16

# --- 2. 전역 변수 및 상수 ---

PPR = 95.0
left_rpm = 0.0
right_rpm = 0.0
last_pulse_time_left = time.monotonic()
last_pulse_time_right = time.monotonic()
RPM_TIMEOUT = 0.2

gps_data = {
    'time': 'N/A', 'status': 'V',
    'latitude': 'N/A', 'longitude': 'N/A'
}

line_follow_mode = False
robot_stop = False

data_lock = threading.Lock()

# ⬇️⬇️⬇️ [수정] PWM 객체 대신 pigpio 인스턴스 사용 ⬇️⬇️⬇️
pi = None
# ⬆️⬆️⬆️ [수정] 완료 ⬆️⬆️⬆️

gps_serial = None
bt_serial = None

# --- 3. 하드웨어 초기화 ---

# ⬇️⬇️⬇️ [수정] setup_gpio 함수 전체 변경 ⬇️⬇️⬇️
def setup_gpio():
    global pi

    # pigpio 데몬에 연결
    pi = pigpio.pi()
    if not pi.connected:
        print("❌ pigpio 데몬에 연결할 수 없습니다.")
        print("  (sudo systemctl start pigpiod 를 실행했는지 확인하세요.)")
        exit()

    # 모터 핀 초기화 (출력)
    motor_pins = [L_FRONT_PWM_PIN, L_FRONT_DIR_PIN, L_REAR_PWM_PIN, L_REAR_DIR_PIN,
                  R_FRONT_PWM_PIN, R_FRONT_DIR_PIN, R_REAR_PWM_PIN, R_REAR_DIR_PIN]
    for pin in motor_pins:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0) # Low로 초기화

    # 모터 하드웨어 PWM 설정
    pwm_pins = [L_FRONT_PWM_PIN, L_REAR_PWM_PIN, R_FRONT_PWM_PIN, R_REAR_PWM_PIN]
    for pin in pwm_pins:
        # 하드웨어 PWM 주파수 설정 (1000Hz)
        pi.set_PWM_frequency(pin, 1000)
        # PWM 범위(Range)를 255로 설정 (0~255 값으로 제어)
        pi.set_PWM_range(pin, 255)
        # 듀티 사이클 0으로 시작
        pi.set_PWM_dutycycle(pin, 0)

    # 적외선 센서 핀 초기화 (입력, 풀다운)
    sensor_pins = [LEFT_SENSOR_PIN, CENTER_SENSOR_PIN, RIGHT_SENSOR_PIN]
    for pin in sensor_pins:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_DOWN)

    # 엔코더 핀 초기화 (입력, 풀업)
    encoder_dir_pins = [LEFT_ENCODER_DIR_PIN, RIGHT_ENCODER_DIR_PIN]
    for pin in encoder_dir_pins:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)

    encoder_pulse_pins = [LEFT_ENCODER_PULSE_PIN, RIGHT_ENCODER_PULSE_PIN]
    for pin in encoder_pulse_pins:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_UP)

    # 엔코더 인터럽트(콜백) 설정
    pi.callback(LEFT_ENCODER_PULSE_PIN, pigpio.RISING_EDGE, left_encoder_callback)
    pi.callback(RIGHT_ENCODER_PULSE_PIN, pigpio.RISING_EDGE, right_encoder_callback)

# ⬆️⬆️⬆️ [수정] setup_gpio 함수 완료 ⬆️⬆️⬆️

def setup_uart():
    # (기존과 동일)
    global gps_serial
    try:
        gps_serial = serial.Serial('/dev/ttyS0', 9600, timeout=1)
        print("✅ GPS UART(/dev/ttyS0) 연결 성공")
    except Exception as e:
        print(f"❌ GPS UART 연결 실패: {e}")
        print("  (raspi-config에서 Serial Port를 활성화했는지 확인하세요.)")

def setup_bluetooth_uart():
    # (기존과 동일)
    global bt_serial
    try:
        bt_serial = serial.Serial('/dev/rfcomm0', 9600, timeout=1)
        print("✅ Bluetooth UART(/dev/rfcomm0) 연결 대기 중...")
    except Exception as e:
        print(f"❌ Bluetooth UART 연결 실패: {e}")
        print("  (휴대폰이 연결되었는지, 'sudo rfcomm watch'가 실행 중인지 확인하세요.)")

# ⬇️⬇️⬇️ [수정] cleanup 함수 변경 ⬇️⬇️⬇️
def cleanup():
    print("\n프로그램 종료. GPIO 정리 중...")
    global robot_stop, pi
    robot_stop = True

    if gps_serial and gps_serial.is_open:
        gps_serial.close()
    if bt_serial and bt_serial.is_open:
        bt_serial.close()

    if pi and pi.connected:
        # 모든 모터 정지 (PWM 0으로 설정)
        set_motor_speed(0, 0, 1, 1)

        # pigpio 연결 해제
        pi.stop()

    print("✅ GPIO 정리 완료.")
# ⬆️⬆️⬆️ [수정] cleanup 함수 완료 ⬆️⬆️⬆️


# --- 4. 엔코더 콜백 (ISR 대용) ---

# ⬇️⬇️⬇️ [수정] pigpio 콜백 시그니처 및 GPIO.input 변경 ⬇️⬇️⬇️
def left_encoder_callback(gpio, level, tick):
    """(수정) 펄스 주기 측정 방식으로 RPM 즉시 계산"""
    global left_rpm, last_pulse_time_left, pi

    current_time = time.monotonic()
    period = current_time - last_pulse_time_left

    if period > 0.0001:
        frequency = 1.0 / period
        rps = frequency / PPR
        rpm = rps * 60.0

        # [수정] GPIO.input -> pi.read
        if not pi.read(LEFT_ENCODER_DIR_PIN):
            rpm = -rpm

        with data_lock:
            left_rpm = rpm

    last_pulse_time_left = current_time

def right_encoder_callback(gpio, level, tick):
    """(수정) 펄스 주기 측정 방식으로 RPM 즉시 계산"""
    global right_rpm, last_pulse_time_right, pi

    current_time = time.monotonic()
    period = current_time - last_pulse_time_right

    if period > 0.0001:
        frequency = 1.0 / period
        rps = frequency / PPR
        rpm = rps * 60.0

        # [수정] GPIO.input -> pi.read
        if not pi.read(RIGHT_ENCODER_DIR_PIN):
            rpm = -rpm

        with data_lock:
            right_rpm = rpm

    last_pulse_time_right = current_time
# ⬆️⬆️⬆️ [수정] 엔코더 콜백 완료 ⬆️⬆️⬆️


# --- 5. 모터 제어 ---

# ⬇️⬇️⬇️ [수정] set_motor_speed 함수 전체 변경 ⬇️⬇️⬇️
def set_motor_speed(left_pwm, right_pwm, left_dir, right_dir):
    global pi

    # PWM 범위(0~255)에 맞게 값 제한
    left_dc = max(0, min(255, left_pwm))
    right_dc = max(0, min(255, right_pwm))

    # 왼쪽 모터 방향 설정 (GPIO.output -> pi.write)
    pi.write(L_FRONT_DIR_PIN, left_dir)
    pi.write(L_REAR_DIR_PIN, left_dir)
    # 왼쪽 모터 속도 설정 (ChangeDutyCycle -> set_PWM_dutycycle)
    pi.set_PWM_dutycycle(L_FRONT_PWM_PIN, left_dc)
    pi.set_PWM_dutycycle(L_REAR_PWM_PIN, left_dc)

    # 오른쪽 모터 방향 설정
    pi.write(R_FRONT_DIR_PIN, right_dir)
    pi.write(R_REAR_DIR_PIN, right_dir)
    # 오른쪽 모터 속도 설정
    pi.set_PWM_dutycycle(R_FRONT_PWM_PIN, right_dc)
    pi.set_PWM_dutycycle(R_REAR_PWM_PIN, right_dc)
# ⬆️⬆️⬆️ [수정] 모터 제어 함수 완료 ⬆️⬆️⬆️

# --- 6. 라인 트레이싱 로직 ---

# ⬇️⬇️⬇️ [수정] line_follow_logic 함수 (GPIO.input -> pi.read) ⬇️⬇️⬇️
def line_follow_logic():
    global pi

    # pigpio는 LOW=0, HIGH=1을 반환. (LOW일 때 라인 감지 가정)
    L = 1 if pi.read(LEFT_SENSOR_PIN) == 0 else 0
    C = 1 if pi.read(CENTER_SENSOR_PIN) == 0 else 0
    R = 1 if pi.read(RIGHT_SENSOR_PIN) == 0 else 0

    BASE_SPEED = 120
    TURN_SPEED = 50

    if C == 1 and L == 0 and R == 0:
        set_motor_speed(BASE_SPEED, BASE_SPEED, 1, 1) # 직진
    elif L == 1 and C == 0 and R == 0:
        set_motor_speed(TURN_SPEED, BASE_SPEED, 1, 1) # 좌회전
    elif L == 0 and C == 0 and R == 1:
        set_motor_speed(BASE_SPEED, TURN_SPEED, 1, 1) # 우회전
    elif L == 1 and C == 1 and R == 0:
        set_motor_speed(0, BASE_SPEED, 1, 1) # 더 좌회전
    elif L == 0 and C == 1 and R == 1:
        set_motor_speed(BASE_SPEED, 0, 1, 1) # 더 우회전
    else:
        set_motor_speed(0, 0, 1, 1) # 정지
# ⬆️⬆️⬆️ [수정] 라인 트레이싱 함수 완료 ⬆️⬆️⬆️

# --- 7. 스레드 함수 ---
# (rpm_watchdog_thread - 변경 없음)
# (parse_nmea_sentence - 변경 없음)
# (gps_reader_thread - 변경 없음)
# (bluetooth_command_thread - 변경 없음)

def rpm_watchdog_thread():
    """(신규) RPM 타임아웃 감시 스레드"""
    global left_rpm, right_rpm

    while not robot_stop:
        time.sleep(0.05)
        current_time = time.monotonic()

        with data_lock:
            if (current_time - last_pulse_time_left > RPM_TIMEOUT) and (left_rpm != 0.0):
                left_rpm = 0.0

            if (current_time - last_pulse_time_right > RPM_TIMEOUT) and (right_rpm != 0.0):
                right_rpm = 0.0

def parse_nmea_sentence(line):
    # (기존 코드와 100% 동일)
    parts = line.strip().split(',')
    if not parts: return None
    try:
        if parts[0] == "$GNGGA" and len(parts) > 6:
            return {
                'time': parts[1], 'latitude': parts[2], 'longitude': parts[4],
                'status': 'A' if parts[6] != '0' else 'V'
            }
        elif parts[0] == "$GNGLL" and len(parts) > 6:
            return {
                'time': parts[5], 'latitude': parts[1], 'longitude': parts[3],
                'status': parts[6]
            }
        elif parts[0] == "$GPRMC" and len(parts) > 5:
            return {
                'time': parts[1], 'status': parts[2],
                'latitude': parts[3], 'longitude': parts[5]
            }
    except Exception:
        return None
    return None

def gps_reader_thread():
    # (기존 코드와 100% 동일)
    global gps_data
    if not gps_serial: return
    while not robot_stop:
        try:
            line = gps_serial.readline().decode('ascii', errors='replace')
            if line.startswith('$'):
                new_data = parse_nmea_sentence(line)
                if new_data:
                    with data_lock:
                        gps_data.update(new_data)
        except serial.SerialException:
            break
        except Exception:
            pass

def bluetooth_command_thread():
    # (기존 코드와 100% 동일)
    global line_follow_mode
    if not bt_serial: return

    print("\n--- 로봇 제어 (Bluetooth) ---")
    bt_serial.write(b"--- Robot Control Ready ---\r\n")
    bt_serial.write(b" 'cw', 'ccw', 'st', 's100', 'ln', 'm'\r\n")
    print("-------------------")

    while not robot_stop:
        try:
            if bt_serial.in_waiting > 0:
                line = bt_serial.readline().decode('ascii', errors='replace')
                cmd = line.strip().lower()
                if not cmd: continue

                if cmd == 'ln':
                    line_follow_mode = True
                    print("〰 LINE FOLLOW START")
                    bt_serial.write(b"LINE FOLLOW START\r\n")
                elif cmd == 'm' or cmd == 'st':
                    line_follow_mode = False
                    set_motor_speed(0, 0, 1, 1)
                    print("⛔ STOP / MANUAL MODE")
                    bt_serial.write(b"STOP / MANUAL MODE\r\n")
                elif not line_follow_mode:
                    if cmd == 'cw':
                        set_motor_speed(255, 255, 1, 1)
                        print("➡ CW (정방향)")
                        bt_serial.write(b"CW (Forward)\r\n")
                    elif cmd == 'ccw':
                        set_motor_speed(255, 255, 0, 0)
                        print("⬅ CCW (역방향)")
                        bt_serial.write(b"CCW (Reverse)\r\n")
                    elif cmd.startswith('s') and len(cmd) > 1 and cmd[1:].isdigit():
                        speed = int(cmd[1:])
                        speed = max(0, min(255, speed))
                        # ⬇️⬇️⬇️ [수정] 'a' 오타 삭제 및 공백 제거 ⬇️⬇️⬇️
                        set_motor_speed(speed, speed, 1, 1)
                        print(f"⚙ SPEED = {speed}")
                        bt_serial.write(f"SPEED = {speed}\r\n".encode())
                    else:
                        print("⚠ UNKNOWN CMD")
                        bt_serial.write(b"UNKNOWN CMD\r\n")
                else:
                    print("⚠ LINE MODE ON. Use 'm' or 'st' to stop.")
                    bt_serial.write(b"LINE MODE ON. Use 'm' or 'st' to stop.\r\n")

            time.sleep(0.1)

        except serial.SerialException as e:
            print(f"블루투스 연결 끊김: {e}")
            break
        except Exception as e:
            if not robot_stop: print(f"명령 처리 오류: {e}")
            break

# --- 8. 메인 로직 스레드 ---
# (main_logic_thread - 변경 없음)
def main_logic_thread():
    global left_rpm, right_rpm, gps_data

    last_print_time = time.time()
    last_line_check_time = time.time()

    while not robot_stop:
        current_time = time.time()

        if line_follow_mode:
            if current_time - last_line_check_time >= 0.05:
                line_follow_logic()
                last_line_check_time = current_time

        if current_time - last_print_time >= 1.0:
            last_print_time = current_time

            with data_lock:
                l_rpm = left_rpm
                r_rpm = right_rpm
                status = gps_data['status']
                lat = gps_data['latitude']
                lon = gps_data['longitude']

            status_msg = f"[S] L={l_rpm:.1f}, R={r_rpm:.1f} | GPS:{status}, LAT:{lat}\r\n"

            print(status_msg.strip())

            if bt_serial and bt_serial.is_open:
                try:
                    bt_serial.write(status_msg.encode())
                except serial.SerialException:
                    print("BT 쓰기 오류 (연결 끊김)")

        time.sleep(0.01) # 10ms


# --- 9. 메인 프로그램 실행 ---
# (if __name__ == "__main__": - 변경 없음)

if __name__ == "__main__":

    atexit.register(cleanup)

    try:
        setup_gpio()
        setup_uart()             # GPS용
        setup_bluetooth_uart()   # 블루투스용

        if not bt_serial:
             print("Bluetooth 포트 열기 실패. 3초 후 종료합니다.")
             time.sleep(3)
             exit()

        print("휴대폰 앱에서 블루투스 연결을 시도하세요...")

        t_rpm = threading.Thread(target=rpm_watchdog_thread, daemon=True)
        t_gps = threading.Thread(target=gps_reader_thread, daemon=True)
        t_cmd = threading.Thread(target=bluetooth_command_thread)
        t_main = threading.Thread(target=main_logic_thread, daemon=True)

        t_rpm.start()
        t_gps.start()
        t_cmd.start()
        t_main.start()

        t_cmd.join()

    except KeyboardInterrupt:
        print("\nCtrl+C 감지. 프로그램 종료 중...")
    except Exception as e:
        print(f"알 수 없는 치명적 오류 발생: {e}")
    finally:
        cleanup()