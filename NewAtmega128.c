/*
 * IRsensorGPS2_Restored.c
 *
 * [수정 사항]
 * 1. 모터 드라이버: MDD10A (DIR 1핀 제어) 적용
 * 2. 엔코더: Input Capture (Timer1, Timer3) 적용
 * 3. 나머지 모든 코드는 사용자 원본 스타일 유지
 */ 

#define F_CPU 16000000UL 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// ─────────────── 1. 핀 정의 (MDD10A + Timer0/2 사용) ───────────────
// [중요] Timer0(PB4)와 Timer2(PB7)을 PWM으로 쓰기 위해 핀 배치 조정
// 원래 코드의 DIR1, DIR3 이름 유지
#define DIR1 PB5  // 왼쪽 모터 방향 (기존 PWM 핀 재활용)
// #define DIR2 PB6 // (MDD10A는 1핀 제어이므로 사용 안 함)

#define DIR3 PA3  // 오른쪽 모터 방향
// #define DIR4 PA4 // (MDD10A는 1핀 제어이므로 사용 안 함)

// [적외선 센서] (원본 유지)
#define SENSOR_PORT PINA       
#define LEFT_SENSOR (1<<PA0)   
#define CENTER_SENSOR (1<<PA1) 
#define RIGHT_SENSOR (1<<PA2)  

// [엔코더 핀] (Input Capture 핀으로 설정)
#define LEFT_ENCODER_PULSE PE7    // Timer3 ICP3
#define LEFT_ENCODER_DIR PE5      
#define RIGHT_ENCODER_PULSE PD4   // Timer1 ICP1 (★주의: 배선 변경 필수)
#define RIGHT_ENCODER_DIR PD5     

#define PPR 95  

// ─────────────── 2. 변수 정의 (Input Capture용 수정) ───────────────
volatile long left_pulse_count = 0; 
volatile int left_direction = 1;     
volatile uint32_t left_ovf = 0;           // Overflow 카운트
volatile uint32_t left_period_ticks = 0;  // 주기 Ticks
volatile uint8_t left_new_pulse = 0;      
volatile uint16_t left_no_pulse_timer = 0; 
double left_rpm = 0;               

volatile long right_pulse_count = 0; 
volatile int right_direction = 1;    
volatile uint32_t right_ovf = 0;          // Overflow 카운트
volatile uint32_t right_period_ticks = 0; // 주기 Ticks
volatile uint8_t right_new_pulse = 0; 
volatile uint16_t right_no_pulse_timer = 0; 
double right_rpm = 0;                

// [GPS 변수] (원본 유지)
#define GPS_BUFFER_SIZE 100 
volatile char gps_rx_buffer[GPS_BUFFER_SIZE];
volatile uint8_t gps_rx_idx = 0;
volatile uint8_t gps_data_ready = 0; 

struct {
    char time[12];
    char status; 
    char latitude[12];
    char longitude[13];
} gps_data;

// ─────────────── 3. 통신 함수 (원본 복원) ───────────────
void uart1_init()
{
    UBRR1H = 0;
    UBRR1L = 8; 
    UCSR1B = (1 << RXEN1) | (1 << TXEN1); 
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); 
}
void uart1_tx(char data)
{
    while (!(UCSR1A & (1 << UDRE1))); 
    UDR1 = data; 
}
char uart1_rx()
{
    while (!(UCSR1A & (1 << RXC1))); 
    return UDR1; 
}
void uart1_print(const char *str) // [복원] 함수명 원복
{
    while (*str) uart1_tx(*str++); 
}

void uart0_init()
{
    UBRR0H = (uint8_t)(103 >> 8);
    UBRR0L = (uint8_t)103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// ─────────────── 4. GPS 파싱 (Switch문 원본 복원) ───────────────
void parse_gprmc(const char *buffer)
{
    if (strncmp(buffer, "$GPRMC", 6) != 0) return;
    char temp_buffer[GPS_BUFFER_SIZE]; strcpy(temp_buffer, buffer);
    char *token; char *saveptr = NULL; int field_count = 0;

    token = strtok_r(temp_buffer, ",", &saveptr);
    while ((token = strtok_r(NULL, ",", &saveptr)) != NULL)
    {
        field_count++;
        switch (field_count) {
            case 1: strncpy(gps_data.time, token, sizeof(gps_data.time)-1); break;
            case 2: gps_data.status = token[0]; break;
            case 3: strncpy(gps_data.latitude, token, sizeof(gps_data.latitude)-1); break;
            case 5: strncpy(gps_data.longitude, token, sizeof(gps_data.longitude)-1); return;
            default: break;
        }
    }
}

void parse_gngga(const char *buffer)
{
    if (strncmp(buffer, "$GNGGA", 6) != 0) return;
    char temp_buffer[GPS_BUFFER_SIZE]; strcpy(temp_buffer, buffer);
    char *token; char *saveptr = NULL; int field_count = 0;

    token = strtok_r(temp_buffer, ",", &saveptr); 
    while ((token = strtok_r(NULL, ",", &saveptr)) != NULL)
    {
        field_count++;
        switch (field_count) {
            case 1: strncpy(gps_data.time, token, sizeof(gps_data.time)-1); break;
            case 2: strncpy(gps_data.latitude, token, sizeof(gps_data.latitude)-1); break;
            case 4: strncpy(gps_data.longitude, token, sizeof(gps_data.longitude)-1); break;
            case 6: if (token[0] == '0') gps_data.status = 'V'; else gps_data.status = 'A'; return;
            default: break;
        }
    }
}

void parse_gngll(const char *buffer)
{
    if (strncmp(buffer, "$GNGLL", 6) != 0) return;
    char temp_buffer[GPS_BUFFER_SIZE]; strcpy(temp_buffer, buffer);
    char *token; char *saveptr = NULL; int field_count = 0;

    token = strtok_r(temp_buffer, ",", &saveptr); 
    while ((token = strtok_r(NULL, ",", &saveptr)) != NULL)
    {
        field_count++;
        switch (field_count) {
            case 1: strncpy(gps_data.latitude, token, sizeof(gps_data.latitude)-1); break;
            case 3: strncpy(gps_data.longitude, token, sizeof(gps_data.longitude)-1); break;
            case 5: strncpy(gps_data.time, token, sizeof(gps_data.time)-1); break;
            case 6: gps_data.status = token[0]; return;
            default: break;
        }
    }
}

// [복원] GPS 인터럽트 로직 ( \r\n 처리 포함 )
ISR(USART0_RX_vect)
{
	char received_char = UDR0;
	if (received_char == '$') {
		gps_rx_idx = 0; 
	}
	if (gps_rx_idx < GPS_BUFFER_SIZE - 1) {
		gps_rx_buffer[gps_rx_idx++] = received_char;
		gps_rx_buffer[gps_rx_idx] = '\0'; 
	}
	
	if (received_char == '\n') {
		if (gps_rx_idx > 0) {
			if (gps_rx_buffer[gps_rx_idx-1] == '\n') {
				gps_rx_buffer[gps_rx_idx-1] = '\0'; 
				if (gps_rx_idx > 1 && gps_rx_buffer[gps_rx_idx-2] == '\r') {
					gps_rx_buffer[gps_rx_idx-2] = '\0'; 
				}
			}
			gps_data_ready = 1; 
		}
	}
}

// ─────────────── 5. 초기화 함수 (수정됨) ───────────────
void sensor_init(void)
{
    DDRA &= ~((1<<PA0) | (1<<PA1) | (1<<PA2));
    PORTA &= ~((1<<PA0) | (1<<PA1) | (1<<PA2));
}

void motor_init(void)
{
    // [수정] MDD10A용 핀 설정 (PWM + DIR 1개)
    // 왼쪽: PWM(PB4), DIR1(PB5)
    DDRB |= (1<<PB4) | (1<<DIR1);
    // 오른쪽: PWM(PB7), DIR3(PA3)
    DDRB |= (1<<PB7);
    DDRA |= (1<<DIR3);

    // Timer0 (Left PWM): Fast PWM, Prescaler 8
    TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS01);
    // Timer2 (Right PWM): Fast PWM, Prescaler 8
    TCCR2 = (1<<WGM20) | (1<<WGM21) | (1<<COM21) | (1<<CS21);

    OCR0 = 0;
    OCR2 = 0;
}

void encoder_init(void)
{
    // 핀 입력 설정
    DDRE &= ~((1<<LEFT_ENCODER_PULSE) | (1<<LEFT_ENCODER_DIR));
    PORTE |= (1<<LEFT_ENCODER_PULSE) | (1<<LEFT_ENCODER_DIR);

    DDRD &= ~((1<<RIGHT_ENCODER_PULSE) | (1<<RIGHT_ENCODER_DIR));
    PORTD |= (1<<RIGHT_ENCODER_PULSE) | (1<<RIGHT_ENCODER_DIR);

    // Timer3 (Left): Input Capture, No Prescaling (16MHz)
    TCCR3A = 0; 
    TCCR3B = (1 << ICES3) | (1 << CS30);
    ETIMSK |= (1 << TICIE3) | (1 << TOIE3); 
    TCNT3 = 0;

    // Timer1 (Right): Input Capture, No Prescaling (16MHz)
    TCCR1A = 0; 
    TCCR1B = (1 << ICES1) | (1 << CS10);
    TIMSK |= (1 << TICIE1) | (1 << TOIE1); 
    TCNT1 = 0;

    sei(); 
}

// ─────────────── 6. 엔코더 인터럽트 (Input Capture) ───────────────

// [왼쪽] Timer 3
ISR(TIMER3_OVF_vect) { left_ovf++; }
ISR(TIMER3_CAPT_vect) {
    uint16_t cap = ICR3;
    uint32_t now = ((uint32_t)left_ovf << 16) | cap;
    static uint32_t prev_l = 0;

    left_period_ticks = now - prev_l;
    prev_l = now;

    if(PINE & (1<<LEFT_ENCODER_DIR)) left_direction = 1;
    else left_direction = -1;

    left_pulse_count += left_direction;
    left_new_pulse = 1;
    left_no_pulse_timer = 0;
}

// [오른쪽] Timer 1
ISR(TIMER1_OVF_vect) { right_ovf++; }
ISR(TIMER1_CAPT_vect) {
    uint16_t cap = ICR1;
    uint32_t now = ((uint32_t)right_ovf << 16) | cap;
    static uint32_t prev_r = 0;

    right_period_ticks = now - prev_r;
    prev_r = now;

    if(PIND & (1<<RIGHT_ENCODER_DIR)) right_direction = 1;
    else right_direction = -1;

    right_pulse_count += right_direction;
    right_new_pulse = 1;
    right_no_pulse_timer = 0;
}

// ─────────────── 7. 모터 제어 및 로직 (MDD10A 적용) ───────────────
void set_motor_speed(int left_pwm, int right_pwm, uint8_t left_dir, uint8_t right_dir)
{
    OCR0 = (uint8_t)abs(left_pwm);    
    OCR2 = (uint8_t)abs(right_pwm);   

    // [MDD10A] 왼쪽 방향 (DIR1: PB5)
    if (left_dir == 1) PORTB |= (1<<DIR1);
    else               PORTB &= ~(1<<DIR1);

    // [MDD10A] 오른쪽 방향 (DIR3: PA3)
    if (right_dir == 1) PORTA |= (1<<DIR3);
    else                PORTA &= ~(1<<DIR3);
}

void line_follow_logic(void) // [복원] 함수명 원복
{
    uint8_t sensor_value = SENSOR_PORT & (LEFT_SENSOR | CENTER_SENSOR | RIGHT_SENSOR);
    uint8_t L = !((sensor_value & LEFT_SENSOR) ? 1 : 0);
    uint8_t C = !((sensor_value & CENTER_SENSOR) ? 1 : 0);
    uint8_t R = !((sensor_value & RIGHT_SENSOR) ? 1 : 0);

    const int BASE_SPEED = 120;
    const int TURN_SPEED = 50;    

    if (C == 0 && L == 1 && R == 1) {
        set_motor_speed(BASE_SPEED, BASE_SPEED, 1, 1);
        uart1_print("straight\r\n");
    }
    else if (L == 0 && C == 1 && R == 1) {
        set_motor_speed(TURN_SPEED, BASE_SPEED, 1, 1);
        uart1_print("left\r\n");
    }
    else if (L == 1 && C == 1 && R == 0) {
        set_motor_speed(BASE_SPEED, TURN_SPEED, 1, 1);
        uart1_print("right\r\n");
    }
    else if (L == 0 && C == 0 && R == 1) {
        set_motor_speed(0, BASE_SPEED, 1, 1);
        uart1_print("more left\r\n");
    }
    else if (L == 1 && C == 0 && R == 0) {
        set_motor_speed(BASE_SPEED, 0, 1, 1);
        uart1_print("more right\r\n");
    }
    else {
        set_motor_speed(0, 0, 1, 1);
        uart1_print("warning\r\n");
    }
}

// 유틸리티 함수 (원본 복원)
static void trim(char *s)
{
    int i = 0; while (s[i]==' ' || s[i]=='\t') i++;
    if (i) memmove(s, s+i, strlen(s+i)+1);
    int n = strlen(s);
    while (n>0 && (s[n-1]==' ' || s[n-1]=='\t')) { s[n-1]='\0'; n--; }
}

static void process_command(char *cmd)
{
    trim(cmd);
    for (char *p = cmd; *p; ++p) *p = (char)tolower((unsigned char)*p);

    if (strcmp(cmd, "cw") == 0) { set_motor_speed(200, 200, 1, 0); uart1_print("CW\r\n"); return; }
    if (strcmp(cmd, "ccw") == 0) { set_motor_speed(200, 200, 0, 1); uart1_print("CCW\r\n"); return; }
    if (strcmp(cmd, "st") == 0) { set_motor_speed(0, 0, 1, 1); uart1_print("STOP\r\n"); return; }
    if (strcmp(cmd, "ln") == 0) { uart1_print("LINE FOLLOW START\r\n"); return; }

    if (cmd[0]=='s') {
        int raw = atoi(cmd+1);
        int val = (strlen(cmd) < 4) ? raw * 10 : raw;
        if (val > 255) val = 255;
        set_motor_speed(val, val, 1, 1);
        uart1_print("SPEED SET\r\n");
    }
}

// ─────────────── 8. MAIN ───────────────
int main(void)
{
    char cmd[16] = {0};
    uint8_t idx = 0;
    uint16_t cmd_idle = 0;
    const uint16_t CMD_IDLE_THRESH = 30;
    
    volatile uint8_t line_follow_mode = 0;

    char msg[100]; 
    char left_rpm_str[20];
    char right_rpm_str[20];
    uint16_t print_timer = 0;
    uint16_t line_check_timer = 0;
    
    motor_init();
    sensor_init();
    encoder_init(); 
    uart1_init();
    uart0_init();

    uart1_print("✅ READY: MDD10A + Input Capture Mode\r\n");
    set_motor_speed(0, 0, 1, 1);

    while(1)
    {
        while (UCSR1A & (1<<RXC1))
        {
            char c = uart1_rx();
            if (c=='\r' || c=='\n' || c=='\t') {}
            else if (c == 'l' || c == 'L') line_follow_mode = 1;
            else if (c == 'm' || c == 'M') {
                line_follow_mode = 0;
                set_motor_speed(0, 0, 1, 1);
                uart1_print("Change to manual mod.\r\n");
            }
            else {
                if (idx < sizeof(cmd)-1) cmd[idx++] = c;
                cmd[idx] = '\0';
            }
            cmd_idle = 0;
        }

        if (idx > 0 && cmd_idle >= CMD_IDLE_THRESH)
        {
            if (strcmp(cmd, "ln") == 0) line_follow_mode = 1;
            else if (strcmp(cmd, "st") == 0) line_follow_mode = 0;
            
            if (!line_follow_mode) process_command(cmd);
            else uart1_print("⚠ LINE MODE ON.\r\n");
            
            idx = 0; cmd[0] = '\0'; cmd_idle = 0;
        }

        if (line_follow_mode)
        {
            if (line_check_timer >= 50) {
                line_follow_logic();
                line_check_timer = 0;
            }
        }

        _delay_ms(10);
        left_no_pulse_timer += 10;
        right_no_pulse_timer += 10;
        print_timer += 10;
        cmd_idle += 10;
        line_check_timer += 10;

        // [RPM 계산]
        if(left_new_pulse) {
            left_new_pulse = 0;
            uint32_t ticks;
            cli(); ticks = left_period_ticks; sei();
            if(ticks > 0) left_rpm = (16000000.0 / (double)ticks) / PPR * 60.0;
        }
        if(left_no_pulse_timer > 200) left_rpm = 0;

        if(right_new_pulse) {
            right_new_pulse = 0;
            uint32_t ticks;
            cli(); ticks = right_period_ticks; sei();
            if(ticks > 0) right_rpm = (16000000.0 / (double)ticks) / PPR * 60.0;
        }
        if(right_no_pulse_timer > 200) right_rpm = 0;

        // [출력]
        if(print_timer >= 1000)
        {
            print_timer = 0;
            dtostrf(left_rpm, 6, 2, left_rpm_str);
            dtostrf(right_rpm, 6, 2, right_rpm_str);

            // [DIR 정보 포함하여 출력]
            char local_gps[GPS_BUFFER_SIZE]; uint8_t rdy=0;
            cli(); if(gps_data_ready) { strcpy(local_gps,(char*)gps_rx_buffer); rdy=1; gps_data_ready=0; } sei();

            if(rdy) {
                if (strncmp(local_gps, "$GNGGA", 6) == 0) parse_gngga(local_gps);
                else if (strncmp(local_gps, "$GNGLL", 6) == 0) parse_gngll(local_gps);
                else if (strncmp(local_gps, "$GPRMC", 6) == 0) parse_gprmc(local_gps);

                sprintf(msg, "RPM[L:%s R:%s] DIR[L:%d R:%d] GPS[%c] %s, %s\r\n", 
                        left_rpm_str, right_rpm_str, left_direction, right_direction, 
                        gps_data.status, gps_data.latitude, gps_data.longitude);
            } else {
                sprintf(msg, "RPM[L:%s R:%s] DIR[L:%d R:%d] No GPS\r\n", 
                        left_rpm_str, right_rpm_str, left_direction, right_direction);
            }
            uart1_print(msg);
        }
    }
}

