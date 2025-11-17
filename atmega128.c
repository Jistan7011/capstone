/*
 * IRsensorGPS2.c
 *
 * Created: 2025-11-12 오후 3:10:23
 * Author : dlrud
 */ 
// (주석... 원본과 동일)
// ...

#define F_CPU 16000000UL 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// (모터 핀 정의... 원본과 동일)
// ...
#define DIR1 PB4 
#define DIR2 PB7 
#define DIR3 PA3 
#define DIR4 PA4 

// (적외선 센서 핀 정의... 원본과 동일)
// ...
#define SENSOR_PORT PINA       
#define LEFT_SENSOR (1<<PA0)   
#define CENTER_SENSOR (1<<PA1) 
#define RIGHT_SENSOR (1<<PA2)  

// (엔코더 핀 정의... 원본과 동일)
// ...
#define LEFT_ENCODER_PULSE PE7    
#define LEFT_ENCODER_DIR PE5      
#define RIGHT_ENCODER_PULSE PE6   
#define RIGHT_ENCODER_DIR PD5     
#define PPR 95            

// (엔코더 변수... 원본과 동일)
// ...
volatile long left_pulse_count = 0; 
volatile int left_direction = 1;     
volatile uint32_t left_period_ticks = 0; 
volatile uint8_t left_new_pulse = 0;   
volatile uint16_t left_no_pulse_timer = 0; 
double left_rpm = 0;               

volatile long right_pulse_count = 0; 
volatile int right_direction = 1;    
volatile uint32_t right_period_ticks = 0; 
volatile uint8_t right_new_pulse = 0; 
volatile uint16_t right_no_pulse_timer = 0; 
double right_rpm = 0;                

volatile uint32_t ovf2 = 0;

// (GPS 변수... 원본과 동일)
// ...
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


// (UART1 함수... 원본과 동일)
// ...
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
void uart1_print(const char *str)
{
    while (*str) uart1_tx(*str++); 
}

// (UART0 함수... 원본과 동일)
// ...
void uart0_init()
{
    UBRR0H = (uint8_t)(103 >> 8);
    UBRR0L = (uint8_t)103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// (parse_gprmc 함수... 원본과 동일)
// ...
void parse_gprmc(const char *buffer)
{
    if (strncmp(buffer, "$GPRMC", 6) != 0) return;

    char temp_buffer[GPS_BUFFER_SIZE];
    strcpy(temp_buffer, buffer);
    
    char *token;
    char *saveptr = NULL;
    int field_count = 0;

    token = strtok_r(temp_buffer, ",", &saveptr);

    while ((token = strtok_r(NULL, ",", &saveptr)) != NULL)
    {
        field_count++;

        switch (field_count) {
            case 1:
            strncpy(gps_data.time, token, sizeof(gps_data.time) - 1);
            gps_data.time[sizeof(gps_data.time) - 1] = '\0';
            break;
            case 2:
            gps_data.status = token[0];
            break;
            case 3:
            strncpy(gps_data.latitude, token, sizeof(gps_data.latitude) - 1);
            gps_data.latitude[sizeof(gps_data.latitude) - 1] = '\0';
            break;
            case 5:
            strncpy(gps_data.longitude, token, sizeof(gps_data.longitude) - 1);
            gps_data.longitude[sizeof(gps_data.longitude) - 1] = '\0';
            return;
            default:
            break;
        }
    }
}

// (parse_gngga 함수... 원본과 동일)
// ...
void parse_gngga(const char *buffer)
{
    if (strncmp(buffer, "$GNGGA", 6) != 0) return;

    char temp_buffer[GPS_BUFFER_SIZE];
    strcpy(temp_buffer, buffer);
    
    char *token;
    char *saveptr = NULL;
    int field_count = 0;

    // 첫 번째 토큰($GNGGA)은 버립니다.
    token = strtok_r(temp_buffer, ",", &saveptr);

    while ((token = strtok_r(NULL, ",", &saveptr)) != NULL)
    {
        field_count++;

        switch (field_count) {
            case 1: // 시간
                strncpy(gps_data.time, token, sizeof(gps_data.time) - 1);
                gps_data.time[sizeof(gps_data.time) - 1] = '\0';
                break;
            case 2: // 위도
                strncpy(gps_data.latitude, token, sizeof(gps_data.latitude) - 1);
                gps_data.latitude[sizeof(gps_data.latitude) - 1] = '\0';
                break;
            case 3: // 위도 방향 (N/S)
                break;
            case 4: // 경도
                strncpy(gps_data.longitude, token, sizeof(gps_data.longitude) - 1);
                gps_data.longitude[sizeof(gps_data.longitude) - 1] = '\0';
                break;
            case 5: // 경도 방향 (E/W)
                break;
            case 6: // GPS 상태 (0=Invalid, 1=GPS, 2=DGPS, ...)
                if (token[0] == '0') gps_data.status = 'V'; // Void (신호 없음)
                else gps_data.status = 'A'; // Active (신호 유효)
                return; 
            default:
                break;
        }
    }
}

// ⬇️⬇️⬇️ [수정] GNGLL 파서 함수 추가 ⬇️⬇️⬇️
// ─────────────── GPS 데이터 파싱 함수 (GNGLL) (추가) ───────────────
// $GNGLL,3508.07418,N,12906.22651,E,035655.00,A,A*7D
// 필드 1: 위도
// 필드 3: 경도
// 필드 5: 시간
// 필드 6: 상태 (A=Active, V=Void)
void parse_gngll(const char *buffer)
{
    if (strncmp(buffer, "$GNGLL", 6) != 0) return;

    char temp_buffer[GPS_BUFFER_SIZE];
    strcpy(temp_buffer, buffer);
    
    char *token;
    char *saveptr = NULL;
    int field_count = 0;

    token = strtok_r(temp_buffer, ",", &saveptr); // $GNGLL 버림

    while ((token = strtok_r(NULL, ",", &saveptr)) != NULL)
    {
        field_count++;

        switch (field_count) {
            case 1: // 위도
                strncpy(gps_data.latitude, token, sizeof(gps_data.latitude) - 1);
                gps_data.latitude[sizeof(gps_data.latitude) - 1] = '\0';
                break;
            case 3: // 경도
                strncpy(gps_data.longitude, token, sizeof(gps_data.longitude) - 1);
                gps_data.longitude[sizeof(gps_data.longitude) - 1] = '\0';
                break;
            case 5: // 시간
                strncpy(gps_data.time, token, sizeof(gps_data.time) - 1);
                gps_data.time[sizeof(gps_data.time) - 1] = '\0';
                break;
            case 6: // 상태 (A=Active, V=Void)
                gps_data.status = token[0];
                return; // 필요한 정보 다 얻었으므로 종료
            default:
                break;
        }
    }
}
// ⬆️⬆️⬆️ [수정] GNGLL 파서 함수 추가 완료 ⬆️⬆️⬆️


// (USART0_RX_vect ISR... 원본과 동일)
// ...
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


// (sensor_init... 원본과 동일)
// ...
void sensor_init(void)
{
    DDRA &= ~((1<<PA0) | (1<<PA1) | (1<<PA2));
    PORTA &= ~((1<<PA0) | (1<<PA1) | (1<<PA2));
}


// (motor_init... 원본과 동일)
// ...
void motor_init(void)
{
    DDRB |= (1<<DIR1) | (1<<DIR2) | (1<<PB5) | (1<<PB6);
    DDRE |= (1<<PE3) | (1<<PE4);
    DDRA |= (1<<DIR3) | (1<<DIR4);

    TCCR1A = (1<<WGM10) | (1<<COM1A1) | (1<<COM1B1);
    TCCR1B = (1<<WGM12) | (1<<CS11);

    TCCR3A = (1<<WGM30) | (1<<COM3A1) | (1<<COM3B1);
    TCCR3B = (1<<WGM32) | (1<<CS31);

    OCR1A = 0;
    OCR1B = 0;
    OCR3A = 0;
    OCR3B = 0;
}


// (set_motor_speed... 사용자 원본과 100% 동일)
// ...
void set_motor_speed(int left_pwm, int right_pwm, uint8_t left_dir, uint8_t right_dir)
{
    // [수정 후] 권장 방식 (Timer 1 = 왼쪽, Timer 3 = 오른쪽)
    OCR1A = (uint8_t)abs(left_pwm);    // PB5 (왼쪽 앞)
    OCR1B = (uint8_t)abs(left_pwm);    // PB6 (왼쪽 뒤)
    OCR3A = (uint8_t)abs(right_pwm);   // PE3 (오른쪽 앞)
    OCR3B = (uint8_t)abs(right_pwm);   // PE4 (오른쪽 뒤)

    // 1. 왼쪽 방향(DIR1, DIR2) 제어
    if (left_dir == 1) {
	    PORTB |= (1<<DIR1);   // 왼쪽 앞바퀴 (PB4)
	    PORTB |= (1<<DIR2);   // 왼쪽 뒷바퀴 (PB7)
	    } else {
	    PORTB &= ~(1<<DIR1);
	    PORTB &= ~(1<<DIR2);
    }

    // 2. 오른쪽 방향(DIR3, DIR4) 제어
    if (right_dir == 1) {
	    PORTA |= (1<<DIR3);   // 오른쪽 앞바퀴 (PA3)
	    PORTA |= (1<<DIR4);   // 오른쪽 뒷바퀴 (PA4)
	    } else {
	    PORTA &= ~(1<<DIR3);
	    PORTA &= ~(1<<DIR4);
    }
}


// (encoder_init... 원본과 동일)
// ...
void encoder_init(void)
{
	DDRE &= ~((1<<LEFT_ENCODER_PULSE) | (1<<LEFT_ENCODER_DIR));
	PORTE |= (1<<LEFT_ENCODER_PULSE) | (1<<LEFT_ENCODER_DIR);

	DDRE &= ~(1<<RIGHT_ENCODER_PULSE);
	PORTE |= (1<<RIGHT_ENCODER_PULSE);

	DDRD &= ~(1<<RIGHT_ENCODER_DIR);
	PORTD |= (1<<RIGHT_ENCODER_DIR);

	TCCR2 = (1 << CS22) | (1 << CS20);
	TIMSK |= (1 << TOIE2);

	EICRB = (1 << ISC71) | (1 << ISC70) | (1 << ISC61) | (1 << ISC60);
	EIMSK |= (1 << INT7) | (1 << INT6);

	ovf2 = 0;
	left_no_pulse_timer = 0;
	right_no_pulse_timer = 0;

	sei(); 
}

// (TIMER2_OVF_vect ISR... 원본과 동일)
// ...
ISR(TIMER2_OVF_vect)
{
    ovf2++; 
}


// (INT7_vect ISR... 원본과 동일)
// ...
ISR(INT7_vect)
{
	uint8_t now8;
	uint32_t now_ovf;

	cli(); 
	now8 = TCNT2;    
	now_ovf = ovf2;  
	
	if ((TIFR & (1<<TOV2)) && (now8 < 128))
	{
		now_ovf++;
	}
	sei(); 

	uint32_t now32 = (now_ovf << 8) | now8; 

	static uint32_t prev32 = 0;
	uint32_t diff = now32 - prev32;
	prev32 = now32;

	if(PINE & (1<<LEFT_ENCODER_DIR)) left_direction = 1;
	else left_direction = -1;

	left_pulse_count += left_direction;

	left_period_ticks = diff; 
	left_new_pulse = 1;
	left_no_pulse_timer = 0;
}


// (INT6_vect ISR... 원본과 동일)
// ...
ISR(INT6_vect)
{
	uint8_t now8;
	uint32_t now_ovf;

	cli(); 
	now8 = TCNT2;   
	now_ovf = ovf2; 
	
	if ((TIFR & (1<<TOV2)) && (now8 < 128))
	{
		now_ovf++;
	}
	sei(); 

	uint32_t now32 = (now_ovf << 8) | now8; 

	static uint32_t prev32 = 0;
	uint32_t diff = now32 - prev32;
	prev32 = now32;

	if(PIND & (1<<RIGHT_ENCODER_DIR)) right_direction = 1;
	else right_direction = -1;

	right_pulse_count += right_direction;

	right_period_ticks = diff; 
	right_new_pulse = 1;
	right_no_pulse_timer = 0;
}


// (line_follow_logic... 원본과 동일)
// ...
void line_follow_logic(void)
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
    else
	{
        set_motor_speed(0, 0, 1, 1);
        uart1_print("warning\r\n");
    }
}


// (trim... 원본과 동일)
// ...
static void trim(char *s)
{
    int i = 0; while (s[i]==' ' || s[i]=='\t') i++;
    if (i) memmove(s, s+i, strlen(s+i)+1);

    int n = strlen(s);
    while (n>0 && (s[n-1]==' ' || s[n-1]=='\t')) { s[n-1]='\0'; n--; }
}


// (process_command... 원본과 동일)
// ...
static void process_command(char *cmd)
{
    trim(cmd);

    for (char *p = cmd; *p; ++p)
    *p = (char)tolower((unsigned char)*p);

    if (strcmp(cmd, "cw") == 0)
    {
        set_motor_speed(255, 255, 1, 1);
        uart1_print("➡ CW\r\n");
        return;
    }
    if (strcmp(cmd, "ccw") == 0)
    {
        set_motor_speed(255, 255, 0, 0);
        uart1_print("⬅ CCW\r\n");
        return;
    }
    if (strcmp(cmd, "st") == 0)
    {
        set_motor_speed(0, 0, 1, 1);
        uart1_print("⛔ STOP\r\n");
        return;
    }
    if (strcmp(cmd, "ln") == 0) {
        uart1_print("〰 LINE FOLLOW START\r\n");
        return;
    }

    if (cmd[0]=='s')
    {
        int ok = 1;
        for (int i=1; cmd[i]; ++i)
        if (cmd[i]<'0' || cmd[i]>'9') { ok=0; break; }

        if (ok && strlen(cmd)>=2 && strlen(cmd)<=4)
        {
            int raw = atoi(cmd+1);
            int val;

            if (strlen(cmd) == 2) val = raw * 10;
            else if (strlen(cmd) == 3) val = raw * 10;
            else val = raw;

            if (val < 0) val = 0;
            if (val > 255) val = 255;

            uint8_t current_dir1 = (PORTB & (1<<DIR1))?1:0;
            uint8_t current_dir2 = (PORTB & (1<<DIR2))?1:0;
            set_motor_speed(val, val, current_dir1, current_dir2);

            char msg[32];
            sprintf(msg, "⚙ SPEED = %d\r\n", val);
            uart1_print(msg);
            return;
        }
    }

    uart1_print("⚠ UNKNOWN CMD\r\n");
}



// ─────────────── MAIN [수정] ───────────────
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

    uart1_print("✅ READY: cw / ccw / st / s0~s255 / line (ln:ON, m:OFF)\r\n");
    set_motor_speed(0, 0, 1, 1);

    while(1)
    {
        // (UART1 수신... 원본과 동일)
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
            else
            {
                if (idx < sizeof(cmd)-1) cmd[idx++] = c;
                cmd[idx] = '\0';
            }

            cmd_idle = 0;
        }

        // (명령 처리... 원본과 동일)
        if (idx > 0 && cmd_idle >= CMD_IDLE_THRESH)
        {
            if (strcmp(cmd, "ln") == 0) line_follow_mode = 1;
            else if (strcmp(cmd, "st") == 0) line_follow_mode = 0;
            
            if (!line_follow_mode) {
                process_command(cmd);
                } else {
                uart1_print("⚠ LINE MODE ON. Use st or 'm' key to stop.\r\n");
            }
            
            idx = 0;
            cmd[0] = '\0';
            cmd_idle = 0;
        }

        // (라인 팔로우... 원본과 동일)
        if (line_follow_mode)
        {
            if (line_check_timer >= 50) {
                line_follow_logic();
                line_check_timer = 0;
            }
        }

        _delay_ms(10);
        
        // (타이머 업데이트... 원본과 동일)
        left_no_pulse_timer += 10;
        right_no_pulse_timer += 10;
        
        print_timer += 10;
        cmd_idle += 10;
        line_check_timer += 10; // (사용자 원본 코드 기준)

        // (RPM 계산... 원본과 동일)
        if(left_new_pulse)
        {
            left_new_pulse = 0;
            uint32_t ticks;

            cli();
            ticks = left_period_ticks;
            sei();

            if(ticks > 0)
            {
                double period_sec = (double)ticks / 125000.0; 
                double rps = 1.0 / (period_sec * PPR);
                left_rpm = rps * 60.0;
            }
        }
        if(left_no_pulse_timer > 200)
            left_rpm = 0;

        if(right_new_pulse)
        {
            right_new_pulse = 0;
            uint32_t ticks;

            cli();
            ticks = right_period_ticks;
            sei();

            if(ticks > 0)
            {
                double period_sec = (double)ticks / 125000.0;
                double rps = 1.0 / (period_sec * PPR);
                right_rpm = rps * 60.0;
            }
        }
        if(right_no_pulse_timer > 200)
            right_rpm = 0;


        // ⬇️⬇️⬇️ [수정] GPS 출력 로직 수정 ⬇️⬇️⬇️
        // ──────────────── 출력 [수정] ────────────────
        if(print_timer >= 1000)
        {
            print_timer = 0;
            dtostrf(left_rpm, 6, 2, left_rpm_str);
            dtostrf(right_rpm, 6, 2, right_rpm_str);

            sprintf(msg, "L-RPM=%s, R-RPM=%s, L-DIR=%d, R-DIR=%d", 
                    left_rpm_str, right_rpm_str, left_direction, right_direction);
            uart1_print(msg);

            char local_gps_buffer[GPS_BUFFER_SIZE];
            uint8_t local_data_ready = 0;

            cli(); 
            if (gps_data_ready) {
                strcpy(local_gps_buffer, (const char*)gps_rx_buffer);
                local_data_ready = 1;
                gps_data_ready = 0; 
            }
            sei(); 

            // [수정] GNGLL 문장도 파싱하도록 else if 추가
            if (local_data_ready) {
                
                if (strncmp(local_gps_buffer, "$GNGGA", 6) == 0) {
                    parse_gngga(local_gps_buffer); 
                    sprintf(msg, ", GPS: %c, LAT: %s, LON: %s\r\n",
                    gps_data.status, gps_data.latitude, gps_data.longitude);
                } 
                // ⬇️⬇️⬇️ [수정] GNGLL 확인 로직 추가 ⬇️⬇️⬇️
                else if (strncmp(local_gps_buffer, "$GNGLL", 6) == 0) {
                    parse_gngll(local_gps_buffer); // GNGLL 파서 호출
                    sprintf(msg, ", GPS: %c, LAT: %s, LON: %s\r\n",
                    gps_data.status, gps_data.latitude, gps_data.longitude);
                }
                // ⬆️⬆️⬆️ [수정] 완료 ⬆️⬆️⬆️
                else if (strncmp(local_gps_buffer, "$GPRMC", 6) == 0) {
                    parse_gprmc(local_gps_buffer); 
                    sprintf(msg, ", GPS: %c, LAT: %s, LON: %s\r\n",
                    gps_data.status, gps_data.latitude, gps_data.longitude);
                }
                else {
                    // GNGGA, GNGLL, GPRMC가 아닌 다른 문장(GLGSV 등)은 RAW로 출력
                    sprintf(msg, ", GPS RAW: %s\r\n", local_gps_buffer);
                }
            } else {
                sprintf(msg, "\r\n");
            }
            uart1_print(msg);
        }
        // ⬆️⬆️⬆️ [수정] GPS 출력 로직 수정 완료 ⬆️⬆️⬆️
    }
}