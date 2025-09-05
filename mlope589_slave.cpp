/*         Marlon Lopez mlope589



*          Discussion Section: 028



 *         Assignment: Custom Lab



 *         Exercise Description: This program will allow for Proportional and Integral Control of a servo based attached to a ball valve off pressure readings recieved over SPI. 



 *        



 *         I acknowledge all content contained herein, excluding template or example code, is my own original work.



 *



 *         Demo Link:  https://youtu.be/mFmePYNhHNs



 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "serialATmega-3.h"
#include "timerISR-Fixed.h"
#include "LCD.h"

// SPI pins
#define PIN_SCK 5
#define PIN_MOSI 3
#define PIN_MISO 4
#define PIN_SS 2

#define NUM_TASKS 4
#define GCD_PERIOD 10

typedef struct {
    signed char    state;
    unsigned long  period;
    unsigned long  elapsedTime;
    int (*TickFct)(int);
} _task;

_task tasks[NUM_TASKS];

volatile uint16_t received_adc = 0;
volatile uint8_t  byte_count = 0;
volatile uint8_t  new_data_ready  = 0;
uint16_t processed_adc   = 0;
uint32_t voltage_mv = 0;
long sensor_pressure = 0;

float integralError = 0.0f;
const float dt = 0.1f;
const float Kp = 0.05f;
const float Ki = 0.08f;
const long  TRICKLE_OCR = 2063;
const long  MAX_STEP_OCR  = 209;
const long  setpoint = 20;
const long  tolerance  = 5;
uint16_t  lastOCR  = 1000;
uint8_t system_off = 0;


// SPI ISR
ISR(SPI_STC_vect) {
    uint8_t data = SPDR;
    if (byte_count == 0) {
        if (data == 0x00) {
            received_adc = (uint16_t)data << 8;
            byte_count = 1;
        }
    } else {
        received_adc |= data;
        byte_count = 0;
        if (received_adc <= 1023) {
            new_data_ready = 1;
        }
    }
}

void SPI_slave_init(void) {
    DDRB &= ~(1 << PIN_SCK);
    DDRB &= ~(1 << PIN_MOSI);
    DDRB |=  (1 << PIN_MISO);
    DDRB &= ~(1 << PIN_SS);
    SPCR |=  (1 << SPE) | (1 << SPIE);
}

void PWM_init(void) {
    DDRB |=  (1 << PB1);
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A |= (1 << COM1A1) | (1 << WGM11);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1   = 39999;
    OCR1A  = 1000;
    lastOCR = 1000;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TimerISR(void) {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += GCD_PERIOD;
    }
}

// Task 0: Process SPI data 100ms 
enum DP_States { DP_Wait };
int TickFct_DataProcessing(int state) {
    switch (state) {
        case DP_Wait:
            if (new_data_ready) {
                processed_adc = received_adc;
                voltage_mv    = (uint32_t)processed_adc * 5000 / 1023;
                if (voltage_mv <= 5000) {
                    sensor_pressure = map(processed_adc, 102, 921, 0, 1000);
                } else {
                    serial_println("ERR: V>5.000V, sample dropped");
                }
                new_data_ready = 0;
            }
            return DP_Wait;
        default:
            return DP_Wait;
    }
}

// Task 1 Display on LCD 1000ms 
enum Display_States { Display_Start, Display_Init, Display_Show };
int TickFct_Display(int state) {
    switch (state) {
        case Display_Start:
            return Display_Init;
        case Display_Init:
            lcd_init();
            return Display_Show;
        case Display_Show: {
            uint32_t voltage = (uint32_t)processed_adc * 5000 / 1023;
            int volts = voltage / 1000;
            int mV    = voltage % 1000;
            long angle = map(lastOCR, 1000, 2700, 0, 80); // 1000–1777 for 0–80°
            const char* status = system_off ? "OFF"
                               : (sensor_pressure < setpoint - tolerance) ? "LOW"
                               : (sensor_pressure > setpoint + tolerance) ? "HIGH" : "OK";
            char line1[17], line2[17];
            sprintf(line1, "V:%d.%03d P:%ldPSI", volts, mV, sensor_pressure);
            sprintf(line2, "Status: %s", status);
            lcd_clear();
            lcd_goto_xy(0, 0);
            lcd_write_str(line1);
            lcd_goto_xy(1, 0);
            lcd_write_str(line2);
            return Display_Show;
        }
        default:
            return Display_Start;
    }
}

// Task 2 PI Control  100ms
enum Control_States { PC_Start, Adjust_Servo };
int TickFct_Proportional(int state) {
    float piOut = 0;
    long p = 0;
    float error = 0;
    uint16_t ocrNew = 0;
    switch (state) {
        case PC_Start:
            return Adjust_Servo;
        case Adjust_Servo:
            if (system_off) {
                ocrNew = 2700;
                integralError = 0.0f;
            } else {
                p = sensor_pressure;
                error = (float)setpoint - p;
                if (error <= 0) {
                    ocrNew = 2700;
                    integralError = 0.0f;
                } else {
                    piOut = TRICKLE_OCR + (Kp * error + Ki * integralError);
                    if (piOut < 1000) piOut = 1000;
                    if (piOut > TRICKLE_OCR) piOut = TRICKLE_OCR;
                    integralError += error * dt;
                    ocrNew = (uint16_t)piOut;
                }
                if (ocrNew > 2700) ocrNew = 2700;
                long delta = (long)ocrNew - lastOCR;
                if (delta > MAX_STEP_OCR) ocrNew = lastOCR + MAX_STEP_OCR;
                else if (delta < -MAX_STEP_OCR) ocrNew = lastOCR - MAX_STEP_OCR;
            }
            if (ocrNew != lastOCR) {
                lastOCR = ocrNew;
                OCR1A = lastOCR;
            }
            return Adjust_Servo;
        default:
            return PC_Start;
    }
}

// Task 3: Button 50ms 
enum Button_States { Wait_Press, Wait_Release };
int TickFct_Button(int state) {
    switch (state) {
        case Wait_Press:
            if (!(PINC & (1 << PC5))) return Wait_Release;
            return Wait_Press;
        case Wait_Release:
            if (PINC & (1 << PC5)) {
                system_off = !system_off;
                return Wait_Press;
            }
            return Wait_Release;
        default:
            return Wait_Press;
    }
}

int main(void) {
    serial_init(9600);
    _delay_ms(1000);
    serial_println("SPI Slave Starting...");
    SPI_slave_init();
    PWM_init();
    DDRC &= ~(1 << PC5);
    PORTC |= (1 << PC5);
    sei();

    tasks[0].period =  100;
    tasks[0].state = DP_Wait;
    tasks[0].elapsedTime = 0;
    tasks[0].TickFct = TickFct_DataProcessing;

    tasks[1].period = 1000;
    tasks[1].state = Display_Start;
    tasks[1].elapsedTime =0;
    tasks[1].TickFct = TickFct_Display;

    tasks[2].period = 100;
    tasks[2].state = PC_Start;
    tasks[2].elapsedTime = 0;
    tasks[2].TickFct = TickFct_Proportional;

    tasks[3].period = 50;
    tasks[3].state  = Wait_Press;
    tasks[3].elapsedTime = 0;
    tasks[3].TickFct  = TickFct_Button;

    TimerSet(GCD_PERIOD);
    TimerOn();
    while (1) { }
    return 0;
}
