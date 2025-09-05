/*         Marlon Lopez mlope589



*          Discussion Section: 028



 *         Assignment: Custom Lab



 *         Exercise Description: This program will allow adc readings to be transfered over SPI



 *        



 *         I acknowledge all content contained herein, excluding template or example code, is my own original work.



 *



 *         Demo Link:  https://youtu.be/mFmePYNhHNs



 */
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "serialATmega-3.h"
#include "timerISR-Fixed.h"
#include "spiAVR.h"

#define NUM_TASKS 2

typedef struct _task {
    signed char state;               
    unsigned long period;            
    unsigned long elapsedTime;       
    int (*TickFct)(int);             
} task;

const unsigned long GCD_PERIOD = 10;
const unsigned long ADC_PERIOD = 100;  
const unsigned long SPI_PERIOD = 100; 
task tasks[NUM_TASKS];

uint16_t adc_value = 0;
volatile uint8_t new_adc_ready = 0;


void TimerISR() {
    for (unsigned int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].elapsedTime >= tasks[i].period) {
            tasks[i].state = tasks[i].TickFct(tasks[i].state);
            tasks[i].elapsedTime = 0;
        }
        tasks[i].elapsedTime += GCD_PERIOD;
    }
}

void ADC_init(void) {
    // Set ADC reference to AVCC (5V)
    ADMUX = (1 << REFS0);
    
    // Enable ADC and set prescaler to 128 (125kHz ADC clock at 16MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRB = 0x00;
}

uint16_t ADC_read(uint8_t channel) {
    // Select ADC channel (0-7)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return ADC result
    return ADC;
}

// Task 1: Read ADC from A0
enum ADC_States { ADC_Start, ADC_Read } adc_state;
int TickFct_ReadADC(int state) {
    // State transitions
    switch (state) {
        case ADC_Start:
            state = ADC_Read;
            break;
        case ADC_Read:
            state = ADC_Read;
            break;
        default:
            state = ADC_Start;
            break;
    }

    // State actions
    switch (state) {
        case ADC_Read:
            adc_value = ADC_read(0);  // Read from channel 0 (A0)
            new_adc_ready = 1;        // Signal new data is ready
            
            // Optional: Print ADC value for debugging
            char buf[32];
            sprintf(buf, "ADC Read: %u", adc_value);
            serial_println(buf);
            break;
        default:
            break;
    }
    return state;
}

// Task 2: Send ADC value via SPI
enum SPI_States { SPI_Start, SPI_Send } spi_state;
int TickFct_SendSPI(int state) {
    // State transitions
    switch (state) {
        case SPI_Start:
            state = SPI_Send;
            break;
        case SPI_Send:
            state = SPI_Send;
            break;
        default:
            state = SPI_Start;
            break;
    }

    // State actions
    switch (state) {
        case SPI_Send:
            if (new_adc_ready) {
                // Select slave (pull SS low)
                PORTB &= ~(1 << PIN_SS);
                _delay_us(10);
                
                // Send high byte first
                SPI_SEND((uint8_t)(adc_value >> 8));
                _delay_us(50);
                
                // Send low byte
                SPI_SEND((uint8_t)(adc_value & 0xFF));
                _delay_us(10);
                
                // Deselect slave (pull SS high)
                PORTB |= (1 << PIN_SS);
                
                new_adc_ready = 0;  // Clear flag
                
                // Optional: Print confirmation
                serial_println("SPI Sent");
            }
            break;
        default:
            break;
    }
    return state;
}

int main(void) {
    // Initialize serial communication
    serial_init(9600);
    _delay_ms(1000);
    serial_println("SPI Master Starting...");
    
    // Initialize ADC and SPI
    ADC_init();
    SPI_INIT();
    
    // Set SS high initially (slave not selected)
    PORTB |= (1 << PIN_SS);
    
    // Initialize tasks
    // Task 0: Read ADC
    tasks[0].period = ADC_PERIOD;
    tasks[0].state = ADC_Start;
    tasks[0].elapsedTime = 0;
    tasks[0].TickFct = &TickFct_ReadADC;
    
    // Task 1: Send SPI
    tasks[1].period = SPI_PERIOD;
    tasks[1].state = SPI_Start;
    tasks[1].elapsedTime = 0;
    tasks[1].TickFct = &TickFct_SendSPI;
    
    // Start timer
    TimerSet(GCD_PERIOD);
    TimerOn();
    
    serial_println("Tasks initialized. Starting main loop...");
    
    // Main loop
    while (1) {
        // Timer handles everything
    }
    
    return 0;
}