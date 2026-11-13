#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#define IN1 PD2
#define IN2 PD3
#define IN3 PD4
#define IN4 PD5

#define ENA PB1 // Arduino D9 / OC1A / left axis
#define ENB PB2 // Arduino D10 / OC1B / right axis

#define BAUD 115200UL

typedef enum
{
  leftAxis,
  rightAxis
} AxisControl;

typedef enum
{
  forward,
  backwards
} direction;

/* ---------------- UART0 (ersetzt Serial) ---------------- */

void UART_Init(void)
{
  // Double Speed Mode: UBRR = F_CPU / (8 * BAUD) - 1 = 16 (bei 115200 Baud)
  UCSR0A = (1 << U2X0);
  UBRR0  = (uint16_t)(F_CPU / (8UL * BAUD) - 1);
  UCSR0B = (1 << TXEN0);                    // nur Senden
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);   // 8 Datenbits, 1 Stopbit, keine Paritaet
}

void UART_PutChar(char c)
{
  while (!(UCSR0A & (1 << UDRE0)));         // warten bis Sendepuffer frei
  UDR0 = c;
}

void UART_PrintStr(const char *s)
{
  while (*s)
  {
    UART_PutChar(*s++);
  }
}

void UART_PrintUInt(uint16_t value)
{
  char buf[5];
  uint8_t i = 0;

  if (value == 0)
  {
    UART_PutChar('0');
    return;
  }
  while (value > 0)
  {
    buf[i++] = '0' + (value % 10);
    value /= 10;
  }
  while (i > 0)
  {
    UART_PutChar(buf[--i]);
  }
}

/* ---------------- ADC (ersetzt analogRead) ---------------- */

void ADC_Init(void)
{
  ADMUX  = (1 << REFS0);                                  // AVcc als Referenz
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128 -> 125 kHz
}

uint16_t ADC_Read(uint8_t channel)
{
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // Kanal waehlen (0 = A0, 1 = A1, 2 = A2)
  ADCSRA |= (1 << ADSC);                      // Wandlung starten
  while (ADCSRA & (1 << ADSC));               // warten bis fertig
  return ADC;
}

/* ---------------- Motoren ---------------- */

void motorPins_Init(void)
{
  DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
}

void PWM_Init(void)
{
  DDRB |= (1 << ENA) | (1 << ENB);
  TCCR1A = 0;
  TCCR1B = 0;

  // Timer1 Mode 14: Fast PWM with ICR1 as TOP
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // Prescaler = 1 (16 MHz / 400 = 40 kHz PWM)

  // TOP Value
  ICR1 = 400;

  OCR1A = 0;
  OCR1B = 0;
}

void set_Speed(AxisControl axis, uint16_t speed)
{
  if (axis == leftAxis)
  {
    OCR1A = speed;
  }
  else
  {
    OCR1B = speed;
  }
}

void set_direction(direction dir)
{
  if (dir == forward)
  {
    PORTD |= (1 << IN1) | (1 << IN4);
    PORTD &= ~((1 << IN2) | (1 << IN3));
  }
  else
  {
    PORTD |= (1 << IN2) | (1 << IN4);
    PORTD &= ~((1 << IN1) | (1 << IN3));
  }
}

/* ---------------- Sensoren ---------------- */

// ADC-Kanaele (A0 = ADC0 = PC0, A1 = ADC1 = PC1, A2 = ADC2 = PC2)
const uint8_t IR_SENSOR_FRONT = 0;
const float PARAM_M_FRONT = 19679.42308;
const float PARAM_D_FRONT = 43.96401985;
const float PARAM_K_FRONT = 15;

const uint8_t IR_SENSOR_LEFT = 2;
const float PARAM_M_LEFT = 7331.25;
const float PARAM_D_LEFT = 15.91666667;
const float PARAM_K_LEFT = 5;

const uint8_t IR_SENSOR_RIGHT = 1;
const float PARAM_M_RIGHT = 8143.2;
const float PARAM_D_RIGHT = 21.25957447;
const float PARAM_K_RIGHT = 7;

const uint16_t DRIVE_SPEED   = 400; // 0..400 (ICR1 = 400)
const uint16_t SAFE_DISTANCE = 40;  // front: obstacle if below this
const uint16_t STOP_DISTANCE = 25;  // front: extremely close, stop first
const uint16_t SIDE_BLOCKED  = 20;  // left/right: blocked if below this

int main(void)
{
  UART_Init();
  ADC_Init();
  motorPins_Init();
  PWM_Init();

  while (1)
  {
    UART_PrintStr("Front   Right   Left\n");

    uint16_t ir_sensor_front_raw = ADC_Read(IR_SENSOR_FRONT);
    uint16_t ir_sensor_front_new = (uint16_t)(PARAM_M_FRONT / (ir_sensor_front_raw + PARAM_D_FRONT)) - PARAM_K_FRONT;
    if (ir_sensor_front_new > 150)
      ir_sensor_front_new = 151;
    else if (ir_sensor_front_new < 20)
      ir_sensor_front_new = 19;
    UART_PrintUInt(ir_sensor_front_new);
    UART_PrintStr("      ");

    uint16_t ir_sensor_left_raw = ADC_Read(IR_SENSOR_LEFT);
    uint16_t ir_sensor_left_new = (uint16_t)(PARAM_M_LEFT / (ir_sensor_left_raw + PARAM_D_LEFT)) - PARAM_K_LEFT;
    if (ir_sensor_left_new > 80)
      ir_sensor_left_new = 81;
    else if (ir_sensor_left_new < 10)
      ir_sensor_left_new = 9;
    UART_PrintUInt(ir_sensor_left_new);
    UART_PrintStr("      ");

    uint16_t ir_sensor_right_raw = ADC_Read(IR_SENSOR_RIGHT);
    uint16_t ir_sensor_right_new = (uint16_t)(PARAM_M_RIGHT / (ir_sensor_right_raw + PARAM_D_RIGHT)) - PARAM_K_RIGHT;
    if (ir_sensor_right_new > 80)
      ir_sensor_right_new = 81;
    else if (ir_sensor_right_new < 10)
      ir_sensor_right_new = 9;
    UART_PrintUInt(ir_sensor_right_new);
    UART_PutChar('\n');

    set_direction(forward);

    if (ir_sensor_front_new < STOP_DISTANCE)
    {
      set_Speed(leftAxis, 250);
      set_Speed(rightAxis, 250);
      _delay_ms(100);
    }

    if (ir_sensor_front_new > SAFE_DISTANCE)
    {
      set_Speed(leftAxis, DRIVE_SPEED);
      set_Speed(rightAxis, DRIVE_SPEED);
    }
    else if (ir_sensor_left_new < SIDE_BLOCKED && ir_sensor_right_new < SIDE_BLOCKED)
    {
      set_Speed(leftAxis, 250);
      set_Speed(rightAxis, 250);
    }
    else if (ir_sensor_left_new > ir_sensor_right_new)
    {
      set_Speed(leftAxis, 250);
      set_Speed(rightAxis, DRIVE_SPEED);
    }
    else
    {
      set_Speed(leftAxis, DRIVE_SPEED);
      set_Speed(rightAxis, 250);
    }

    _delay_ms(200);
  }
}