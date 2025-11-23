/*
 * Camera_LineTracer_Final_v3.c
 *
 * [시스템 구성]
 * 1. Main Controller: ATmega128 (16MHz)
 * 2. Vision: Raspberry Pi (UART0, 115200bps)
 * 3. Remote: JMOD-BT-1 (UART1, 115200bps)
 * 4. Motor: MDD10A (DIR 1핀 제어)
 *
 * [기능 업데이트]
 * 1. 수동 정지: 'x' 키 입력 시 정지 및 피드백 전송
 * 2. 상태 출력: RPM과 함께 DIR(방향) 정보 출력 추가
 * 3. 속도 제어: 숫자키 0~9로 즉시 속도 변경 기능 포함
 */ 

#define F_CPU 16000000UL 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// ─────────────── 1. 핀 정의 ───────────────
#define DIR1 PB5  // 왼쪽 모터 방향
#define DIR3 PA3  // 오른쪽 모터 방향

// [제어 변수]
volatile char rpi_command = 'S';     
volatile uint8_t is_auto_mode = 0;   
volatile char bt_command = 'S';      

// [수동 속도 변수]
volatile int manual_speed = 150; 

// [엔코더 핀]
#define LEFT_ENCODER_PULSE PE7    
#define LEFT_ENCODER_DIR PE5      
#define RIGHT_ENCODER_PULSE PD4   
#define RIGHT_ENCODER_DIR PD5      

#define PPR 95  

// ─────────────── 2. 변수 정의 ───────────────
volatile long left_pulse_count = 0; 
volatile int left_direction = 1;      
volatile uint32_t left_ovf = 0;           
volatile uint32_t left_period_ticks = 0;  
volatile uint8_t left_new_pulse = 0;      
volatile uint16_t left_no_pulse_timer = 0; 
double left_rpm = 0;                

volatile long right_pulse_count = 0; 
volatile int right_direction = 1;     
volatile uint32_t right_ovf = 0;          
volatile uint32_t right_period_ticks = 0; 
volatile uint8_t right_new_pulse = 0; 
volatile uint16_t right_no_pulse_timer = 0; 
double right_rpm = 0;                  

// ─────────────── 3. 통신 초기화 ───────────────

void uart1_tx(char data) {
    while (!(UCSR1A & (1 << UDRE1))); 
    UDR1 = data; 
}
void uart1_print(const char *str) {
    while (*str) uart1_tx(*str++); 
}

// [UART0] Pi (115200bps)
void uart0_init_rpi() {
    UBRR0H = 0;
    UBRR0L = 8; 
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// [UART1] BT (115200bps)
void uart1_init_bt() {
    UBRR1H = 0;
    UBRR1L = 8; 
    UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); 
}

// ─────────────── 4. 통신 인터럽트 (기능 추가됨) ───────────────

// [UART0 RX] Pi 명령
ISR(USART0_RX_vect) {
    char received_char = UDR0;
    if (is_auto_mode) {
        if (received_char == 'F' || received_char == 'L' || 
            received_char == 'R' || received_char == 'S') {
            rpi_command = received_char;
        }
    }
}

// [UART1 RX] BT 명령 (정지 키 'x', 속도 0~9, 방향 표시 추가)
ISR(USART1_RX_vect) {
    char rx_char = UDR1;
    char msg_buf[50]; // 버퍼 사이즈 넉넉하게

    // 1. 비상 정지 (Spacebar) - 모든 모드 초기화
    if (rx_char == ' ') { 
        is_auto_mode = 0;
        bt_command = 'S';
        rpi_command = 'S';
        uart1_print("!! EMERGENCY STOP !!\r\n");
    }
    // 2. 자동 모드 (A)
    else if (rx_char == 'A' || rx_char == 'a') {
        is_auto_mode = 1;
        bt_command = 0;
        uart1_print("Mode: AUTO\r\n");
    }
    // 3. 수동 모드 (M)
    else if (rx_char == 'M' || rx_char == 'm') {
        is_auto_mode = 0;
        rpi_command = 'S';
        bt_command = 'S';
        uart1_print("Mode: MANUAL\r\n");
    }
    // 4. 수동 조작 로직
    else if (!is_auto_mode) {
        
        // [수동 정지] 'x' 키 입력 시 정지
        if (rx_char == 'x' || rx_char == 'X') {
            bt_command = 'S'; 
            uart1_print("CMD: x (Manual Stop)\r\n");
        }
        // [즉시 속도 조절] 숫자키 0~9
        else if (rx_char >= '0' && rx_char <= '9') {
            if (rx_char == '0') manual_speed = 255; // Max
            else manual_speed = (rx_char - '0') * 25; // 1=25 ... 9=225
            
            sprintf(msg_buf, "Speed Set: %d\r\n", manual_speed);
            uart1_print(msg_buf);
        }
        // [속도 미세 조정] +, -
        else if (rx_char == '+') {
            manual_speed += 10;
            if (manual_speed > 255) manual_speed = 255;
            sprintf(msg_buf, "Speed UP: %d\r\n", manual_speed);
            uart1_print(msg_buf);
        }
        else if (rx_char == '-') {
            manual_speed -= 10;
            if (manual_speed < 0) manual_speed = 0;
            sprintf(msg_buf, "Speed DOWN: %d\r\n", manual_speed);
            uart1_print(msg_buf);
        }
        // [방향키] W, S, L, R
        else if (rx_char == 'w' || rx_char == 'W' || 
                 rx_char == 's' || rx_char == 'S' ||
                 rx_char == 'l' || rx_char == 'L' || 
                 rx_char == 'r' || rx_char == 'R') {
            
            bt_command = rx_char; 
            sprintf(msg_buf, "CMD: %c (Spd:%d)\r\n", rx_char, manual_speed);
            uart1_print(msg_buf);
        }
    }
}

// ─────────────── 5. 초기화 함수 ───────────────
void sensor_init(void) { } 

void motor_init(void) {
    DDRB |= (1<<PB4) | (1<<DIR1);
    DDRB |= (1<<PB7);
    DDRA |= (1<<DIR3);

    TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);
    TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS21);
    OCR0 = 0; OCR2 = 0;
}

void encoder_init(void) {
    DDRE &= ~((1<<LEFT_ENCODER_PULSE) | (1<<LEFT_ENCODER_DIR));
    PORTE |= (1<<LEFT_ENCODER_PULSE) | (1<<LEFT_ENCODER_DIR);

    DDRD &= ~((1<<RIGHT_ENCODER_PULSE) | (1<<RIGHT_ENCODER_DIR));
    PORTD |= (1<<RIGHT_ENCODER_PULSE) | (1<<RIGHT_ENCODER_DIR);

    TCCR3A = 0; TCCR3B = (1 << ICES3) | (1 << CS30);
    ETIMSK |= (1 << TICIE3) | (1 << TOIE3); TCNT3 = 0;

    TCCR1A = 0; TCCR1B = (1 << ICES1) | (1 << CS10);
    TIMSK |= (1 << TICIE1) | (1 << TOIE1); TCNT1 = 0;
    sei(); 
}

// ─────────────── 6. 엔코더 로직 ───────────────
ISR(TIMER3_OVF_vect) { left_ovf++; }
ISR(TIMER3_CAPT_vect) {
    uint16_t cap = ICR3;
    uint32_t now = ((uint32_t)left_ovf << 16) | cap;
    static uint32_t prev_l = 0;
    left_period_ticks = now - prev_l; prev_l = now;
    if(PINE & (1<<LEFT_ENCODER_DIR)) left_direction = 1; else left_direction = -1;
    left_pulse_count += left_direction; left_new_pulse = 1; left_no_pulse_timer = 0;
}

ISR(TIMER1_OVF_vect) { right_ovf++; }
ISR(TIMER1_CAPT_vect) {
    uint16_t cap = ICR1;
    uint32_t now = ((uint32_t)right_ovf << 16) | cap;
    static uint32_t prev_r = 0;
    right_period_ticks = now - prev_r; prev_r = now;
    if(PIND & (1<<RIGHT_ENCODER_DIR)) right_direction = 1; else right_direction = -1;
    right_pulse_count += right_direction; right_new_pulse = 1; right_no_pulse_timer = 0;
}

// ─────────────── 7. 모터 제어 ───────────────
void set_motor_speed(int left_pwm, int right_pwm, uint8_t left_dir, uint8_t right_dir) {
    OCR0 = (uint8_t)abs(left_pwm);      
    OCR2 = (uint8_t)abs(right_pwm);   
    if (left_dir == 1) PORTB |= (1<<DIR1); else PORTB &= ~(1<<DIR1);
    if (right_dir == 1) PORTA |= (1<<DIR3); else PORTA &= ~(1<<DIR3);
}

void line_follow_logic(void) { // 자동
    const int BASE_SPEED = 120;
    const int TURN_SPEED = 60; 
    switch (rpi_command) {
        case 'F': set_motor_speed(BASE_SPEED, BASE_SPEED, 1, 1); break;
        case 'L': set_motor_speed(TURN_SPEED, BASE_SPEED, 1, 1); break;
        case 'R': set_motor_speed(BASE_SPEED, TURN_SPEED, 1, 1); break;
        case 'S': set_motor_speed(0, 0, 1, 1); break;
        default: set_motor_speed(0, 0, 1, 1); break;
    }
}

void manual_control_logic(void) { // 수동
    switch (bt_command) {
        case 'w': case 'W': set_motor_speed(manual_speed, manual_speed, 1, 1); break; // 전진
        case 's': case 'S': set_motor_speed(manual_speed, manual_speed, 0, 0); break; // 후진
        case 'l': case 'L': set_motor_speed(0, manual_speed, 1, 1); break; // 좌회전
        case 'r': case 'R': set_motor_speed(manual_speed, 0, 1, 1); break; // 우회전
        default: set_motor_speed(0, 0, 1, 1); break; // 정지 ('x'키 누르거나 초기상태)
    }
}

// ─────────────── 8. MAIN ───────────────
int main(void) {
    char msg[80]; // 출력 내용이 길어져서 버퍼 크기 늘림
    char left_rpm_str[10];
    char right_rpm_str[10];
    uint16_t print_timer = 0;
    uint16_t control_timer = 0;
    
    motor_init();
    encoder_init(); 
    
    uart0_init_rpi(); 
    uart1_init_bt();  

    uart1_print("✅ SYSTEM READY\r\n");
    uart1_print("   [x]:Stop, [0~9]:Speed, [Space]:Emerg\r\n");

    set_motor_speed(0, 0, 1, 1);

    while(1) {
        _delay_ms(10);
        left_no_pulse_timer += 10;
        right_no_pulse_timer += 10;
        print_timer += 10;
        control_timer += 10;

        // [RPM 계산]
        if(left_new_pulse) {
            left_new_pulse = 0;
            uint32_t ticks; cli(); ticks = left_period_ticks; sei();
            if(ticks > 0) left_rpm = (16000000.0 / (double)ticks) / PPR * 60.0;
        }
        if(left_no_pulse_timer > 200) left_rpm = 0;

        if(right_new_pulse) {
            right_new_pulse = 0;
            uint32_t ticks; cli(); ticks = right_period_ticks; sei();
            if(ticks > 0) right_rpm = (16000000.0 / (double)ticks) / PPR * 60.0;
        }
        if(right_no_pulse_timer > 200) right_rpm = 0;

        // [제어 실행]
        if (control_timer >= 50) { 
            control_timer = 0;
            if (is_auto_mode) line_follow_logic(); 
            else manual_control_logic(); 
        }

        // [상태 출력] (DIR 정보 추가됨)
        if(print_timer >= 1000) {
            print_timer = 0;
            dtostrf(left_rpm, 4, 1, left_rpm_str);
            dtostrf(right_rpm, 4, 1, right_rpm_str);

            if (is_auto_mode) {
                // 자동 모드: Pi 명령 + DIR + RPM
                sprintf(msg, "[AUTO] CMD:%c DIR[%d:%d] RPM L:%s R:%s\r\n", 
                        rpi_command, left_direction, right_direction, left_rpm_str, right_rpm_str);
            } else {
                // 수동 모드: 속도 + DIR + RPM
                sprintf(msg, "[MANUAL] Spd:%d DIR[%d:%d] RPM L:%s R:%s\r\n", 
                        manual_speed, left_direction, right_direction, left_rpm_str, right_rpm_str);
            }
            
            uart1_print(msg);
        }
    }
}

