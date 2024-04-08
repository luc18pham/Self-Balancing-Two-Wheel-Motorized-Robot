// Frequency Counter / Timer Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for snprintf)

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   SIGNAL_IN on PC6 (WT1CCP0)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

/*

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// PortC masks
#define FREQ_IN_MASK 64

// PortF masks
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


void enableCounterMode()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
	WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
	WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
	WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // count positive edges
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
	WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
}

void disableCounterMode()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off time base timer
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off event counter
    NVIC_DIS0_R = 1 << (INT_TIMER1A-16);            // turn-off interrupt 37 (TIMER1A)
}

void enableTimerMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)
}

void disableTimerMode()
{
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter
    NVIC_DIS3_R = 1 << (INT_WTIMER1A-16-96);        // turn-off interrupt 112 (WTIMER1A)
}

// Frequency counter service publishing latest frequency measurements every second
void timer1Isr()
{
    frequency = WTIMER1_TAV_R;                   // read counter input
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    GREEN_LED ^= 1;                              // status
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;           // clear interrupt flag
}

// Period timer service publishing latest time measurements every positive edge
void wideTimer1Isr()
{
    time = WTIMER1_TAV_R;                        // read counter input
    WTIMER1_TAV_R = 0;                           // zero counter for next edge
    GREEN_LED ^= 1;                              // status
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button

    // Configure SIGNAL_IN for frequency and time measurements
	GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK;                // enable bit 6 for digital input
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------


int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    // Configure selected mode
    if (timeMode)
    {
        disableCounterMode();
        enableTimerMode();
    }
    else
    {
        disableTimerMode();
        enableCounterMode();
    }

    // Use blue LED to show mode
    BLUE_LED = timeMode;

    // Endless loop performing multiple tasks
    char str[40];
    while (true)
    {
        if (timeMode)
            snprintf(str, sizeof(str), "Period:    %7"PRIu32" (us)\n", time / 40);
        else
            snprintf(str, sizeof(str), "Frequency: %7"PRIu32" (Hz)\n", frequency);
        putsUart0(str);

        // debouncing not implemented until keyboard.c example
        // for now, use a delay to allow unique key presses to be detected
    	if (!PUSH_BUTTON)
    	{
    		timeMode = !timeMode;
    		BLUE_LED = timeMode;
    	    if (timeMode)
    	    {
    	        disableCounterMode();
    	        enableTimerMode();
    	    }
    	    else
    	    {
    	        disableTimerMode();
    	        enableCounterMode();
    	    }
    	}
        waitMicrosecond(1000000);
    }
}
*/
