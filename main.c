//Luc Pham
//1001918323

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <math.h>
#include "gpio.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "nvic.h"
#include "i2c0.h"
#include "tm4c123gh6pm.h"

// PortD masks
#define FREQ_IN_MASK 4

uint32_t array[36];
uint8_t count = 0;
uint32_t frequency1 = 0;
uint32_t frequency2 = 0;
char line[100];
uint8_t irIndex = 0;
uint32_t signal = 0b00000000000000000000000000000000;

//Optical Encoder Variables
float time1 = 0;
float time2 = 0;
float sum1 = 0;
float sum2 = 0;
uint8_t time1Index = 0;
uint8_t time2Index = 0;
float time1Array[40];
float time2Array[40];
float T1_avg_bits = 0;
float T2_avg_bits = 0;
float T1_avg = 0;
float T2_avg = 0;
float u_high = 0;
float u_low = 0;

// plant output (y) varies from 0 to 1023 (10 bit range), so place in center
int32_t ySetPoint = 0;

// pid calculation of u
int32_t coeffKp = 10000;
int32_t coeffKi = 85;
int32_t coeffKd = 2;
int32_t coeffKo = 800;
int32_t coeffK = 100000; // denominatr used to scale Kp, Ki, and Kd
int32_t integral = 0;
int32_t iMax = 100000;
int32_t diff;
int32_t error;
float y = 0;
int32_t u = 0;
float deadBand = 25;
float deadBand2 = 0;

//I2C Globals
uint8_t output[14];
uint32_t m = 0;
int16_t accel_x = 0;
int16_t accel_y = 0;
int16_t accel_z = 0;
int16_t gyro_x = 0;
int16_t gyro_y = 0;
int16_t gyro_z = 0;
float accel_x_float = 0;
float accel_y_float = 0;
float accel_z_float = 0;
float gyro_x_float = 0;
float gyro_y_float = 0;
float gyro_z_float = 0;

double sumX = 0;
double sumY = 0;
double sumZ = 0;
double avgBiasX = 0;
double avgBiasY = 0;
double avgBiasZ = 0;
double correctedX = 0;
double correctedY = 0;
double correctedZ = 0;

double pitch = 0;
double roll = 0;
double thetaXGyro = 0;
double confidence = 0;
double confidence2 = 0;
double angle_pitch = 0;
double angle_roll = 0;
uint32_t value1 = 0;
uint32_t value2 = 0;
double gyro_x_ang = 0;
double gyro_y_ang = 0;

void setMotor1Neg(uint16_t speed) //max speed of 1024
{
    PWM0_0_CMPA_R = speed;
}

void setMotor1Pos(uint16_t speed) //max speed of 1024
{
    PWM0_0_CMPB_R = speed;
}

void setMotor2Pos(uint16_t speed) //max speed of 1024
{
    PWM0_3_CMPA_R = speed;
}

void setMotor2Neg(uint16_t speed) //max speed of 1024
{
    PWM0_3_CMPB_R = speed;
}

void enableTimers()
{
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_NEG;           // measure time from positive edge to positive edge
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER3_TAV_R = 0;                               // zero counter for first period
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER3A-16-96);         // turn-on interrupt 112 (WTIMER1A)

    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER5_TAV_R = 0;                               // zero counter for first period
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    NVIC_EN3_R = 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 112 (WTIMER1A)
}

void initPidControllers() //PB0 AND PB1   PB4 and PB5
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2 | SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    // Configure Timer 2A for PID controller
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 40000;                          // set load value to 40000 for 1000 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);              // turn-on interrupt 37 (TIMER1A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer


    // Configure Timer 1A for PID controller
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40000;                          // set load value to 160000 for 250 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}

// Period timer service publishing latest time measurements every negative edge
void wideTimer3Isr()
{
    array[count] = WTIMER3_TAV_R;
    WTIMER3_TAV_R = 0;                           // zero counter for next edge
    count++;
    if(count >= 34)
    {
        if( 486000 <= array[1] && array[1] <= 594000)
        {
            putsUart0("Nec Protocol\n");
            for(irIndex = 0; irIndex <= 35; irIndex++)
            {
                if(2 <= irIndex && irIndex <= 33)
                {
                    if(36000 <= array[irIndex] && array[irIndex] <= 54000 )
                    {
                        signal |= (0<<irIndex);
                    }
                    if(72000 <= array[irIndex] && array[irIndex] <= 108000 )
                    {
                        signal |= (1<<irIndex);
                    }
                }
                sprintf(line,"Array spot %d: %" PRIu32 "\n", irIndex, array[irIndex] );
                putsUart0(line);
                array[irIndex] = 0;
            }
            sprintf(line,"Signal value: %" PRIu32 "\n", signal );
            putsUart0(line);
            putsUart0("\n");
            count = 0;
            irIndex = 0;
            //waitMicrosecond(500);
        }
        else
        {
            for(irIndex = 0; irIndex <= 35; irIndex++)
            {
                array[irIndex] = 0;
            }
            sprintf(line,"Signal value: %" PRIu32 "\n", signal );
            putsUart0(line);
            putsUart0("\n");
            count = 0;
            irIndex = 0;
            //waitMicrosecond(500);
        }
    }
    WTIMER3_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void readGyro(uint8_t data[])
{
    readI2c0Registers(0x68, 0x3B, data, 14);

    accel_x = (output[0]<<8) | output[1];
    accel_x_float = (float)accel_x/4096;

    accel_y = (output[2]<<8) | output[3];
    accel_y_float = (float)accel_y/4096;

    accel_z = (output[4]<<8) | output[5];
    accel_z_float = (float)accel_z/4096;

    gyro_x = (output[8]<<8) + output[9];
    gyro_x_float = (float)gyro_x/65.5;

    gyro_y = (output[10]<<8) + output[11];
    gyro_y_float = (float)gyro_y/65.5;

    gyro_z = (output[12]<<8) + output[13];
    gyro_z_float = (float)gyro_z/65.5;

    pitch = (atan2(accel_x_float, accel_z_float) * 180/3.141592);
    roll = (atan2(accel_y_float, accel_z_float) * 180/3.141592);

    /*
    sprintf(line, "accel_x: %f \n", accel_x_float);
    putsUart0(line);

    sprintf(line, "accel_y: %f \n", accel_y_float);
    putsUart0(line);

    sprintf(line, "accel_z: %f \n", accel_z_float);
    putsUart0(line);

    sprintf(line, "ACCEL MSB X address %d: %d \n", 0, output[0]);
    putsUart0(line);
    sprintf(line, "ACCEL LSB X address %d: %d \n", 1, output[1]);
    putsUart0(line);

    sprintf(line, "ACCEL MSB Y address %d: %d \n", 2, output[2]);
    putsUart0(line);
    sprintf(line, "ACCEL LSB Y address %d: %d \n", 3, output[3]);
    putsUart0(line);

    sprintf(line, "ACCEL MSB Z address %d: %d \n", 4, output[4]);
    putsUart0(line);
    sprintf(line, "ACCEL LSB Z address %d: %d \n", 5, output[5]);
    putsUart0(line);
    */

    /*
    sprintf(line, "GYRO MSB X address %d: %d \n", 8, output[8]);
    putsUart0(line);
    sprintf(line, "GYRO LSB X address %d: %d \n", 9, output[9]);
    putsUart0(line);

    sprintf(line, "GYRO MSB Y address %d: %d \n", 10, output[10]);
    putsUart0(line);
    sprintf(line, "GYRO LSB Y address %d: %d \n", 11, output[11]);
    putsUart0(line);

    sprintf(line, "GYRO MSB Z address %d: %d \n", 12, output[12]);
    putsUart0(line);
    sprintf(line, "GYRO LSB Z address %d: %d \n\n", 13, output[13]);
    putsUart0(line);
    */

    /*
    sprintf(line, "gyro_x: %f \n\n", gyro_x_float);
    putsUart0(line);

    sprintf(line, "gyro_y: %f \n\n", gyro_y_float);
    putsUart0(line);

    sprintf(line, "gyro_z: %f \n\n", gyro_z_float);
    putsUart0(line);
    */
    //confidence = thetaX;
}

void calibrateGyro()
{
    for(m = 0; m < 1000; m++)
    {
        readGyro(output);
        sumX = sumX + gyro_x;
        sumY = sumY + gyro_y;
        sumZ = sumZ + gyro_z;
        //waitMicrosecond(3);
    }
    avgBiasX = sumX / 1000;
    avgBiasY = sumY / 1000;
    avgBiasZ = sumZ / 1000;

    sumX = 0;
    sumY = 0;
    sumZ = 0;
}

void pidIsr2()
{
    readGyro(output);

    correctedX = (double)(gyro_x - avgBiasX); // /65.5 * (1/1000);
    correctedY = (double)(gyro_y - avgBiasY);
    correctedZ = (double)(gyro_z - avgBiasZ);

    angle_roll += correctedX / 65.5 * (1/1000);
    angle_pitch += correctedY / 65.5 *(1/1000);
    //angle_pitch += angle_roll * sin(correctedZ * 0.000002665);
    //angle_roll += angle_pitch * sin(correctedZ * 0.000002665);
    gyro_x_ang = (gyro_x_ang + angle_roll);  //* 180/3.1415;
    gyro_y_ang = (gyro_y_ang + angle_pitch); // * 180/3.1415;

    confidence =  gyro_y_ang * 0.03 + (1 - 0.03) * pitch;
    confidence2 =  gyro_x_ang * 0.95 + (1 - 0.95) * roll;
    gyro_y_ang = confidence;
    gyro_x_ang = confidence2;
    /*
    sprintf(line, "pitch: %f \n\n", pitch);
    putsUart0(line);
    sprintf(line, "angle_pitch: %f \n", angle_pitch);
    putsUart0(line);
    sprintf(line, "angle_roll: %f \n", angle_roll);
    putsUart0(line);
    sprintf(line, "confidence: %f \n", confidence);
    putsUart0(line);
    */
    static int32_t errorLast = 0;

    error = ySetPoint - confidence;
    // determine error (set point - process value, SP-PV)


    // calculate integral and prevent windup
    integral += error;

    int32_t iLimit = iMax * coeffKi;
    if (integral > iLimit)
        integral = iLimit;
    if (integral < -iLimit)
        integral = -iLimit;

    // calculate differential
    diff = error - errorLast;
    errorLast = error;

    // calculate plant input, saturating 10 bit output if needed
    u = (coeffKp * error) + (coeffKi * integral) + (coeffKd * diff);
    u /= coeffK;
    if (u > 1023) u = 1023;
    if (u < -1023) u = -1023;

    if (abs(error) > deadBand)
    {
        /*
        if( error > 80)
        {
            value1 = abs(u) + coeffKo;
            setMotor1Pos(0);
            setMotor2Pos(0);
            setMotor1Neg(value1);
            setMotor2Neg(value1);
        }
        if(error < -80)
        {
            value2 = abs(u) + coeffKo;
            setMotor1Neg(0);
            setMotor2Neg(0);
            setMotor1Pos(value2);
            setMotor2Pos(value2);
        }
        */
        if(u > 0)
        {
            value1 = abs(u) + coeffKo;
            setMotor1Pos(abs(u) + 895);
            setMotor2Pos(abs(u) + 860);
            setMotor1Neg(0);
            setMotor2Neg(0);
        }
        if(u < 0)
        {
            value2 = abs(u) + coeffKo;
            setMotor1Neg(abs(u) + 895);
            setMotor2Neg(abs(u) + 860);
            setMotor1Pos(0);
            setMotor2Pos(0);
        }
    }
    else
    {

        setMotor1Neg(0);
        setMotor2Neg(0);
        setMotor1Pos(0);
        setMotor2Pos(0);

    }

    //sprintf(line, "u: %d, error: %d\r\n", u, error);
    //putsUart0(line);

    /*
    sprintf(line, "error: %d, gyro_y_ang: %f, pitch: %f, u: %d, confidence: %f\r\n", error, gyro_y_ang, pitch, u, confidence);
    putsUart0(line);
    */

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void pidIsr()
{
    y = T1_avg - T2_avg;
    switch(signal)
        {
        case 2891185152: // red up arrow
            if (abs(y) >= deadBand2)
            {
                if(T1_avg >= T2_avg)
                {
                    u_high = 895 + abs(y);
                    u_low = 860 - abs(y);
                    sprintf(line,"u_high: %" PRIi32 " u_low: %" PRIi32 "\n", u_high, u_low );
                    putsUart0(line);
                    sprintf(line,"T1_avg: %f  T2_avg  %f\n", T1_avg, T2_avg);
                    putsUart0(line);
                    putsUart0("\n");
                    setMotor1Pos(u_high);
                    setMotor2Pos(u_low);
                    setMotor1Neg(0);
                    setMotor2Neg(0);
                }

                if(T2_avg > T1_avg)
                {
                    u_high = 860 + abs(y);
                    u_low = 895 - abs(y);
                    sprintf(line,"u_high: %" PRIi32 " u_low: %" PRIi32 "\n", u_high, u_low );
                    putsUart0(line);
                    sprintf(line,"T1_avg: %f  T2_avg  %f\n", T1_avg, T2_avg);
                    putsUart0(line);
                    putsUart0("\n");
                    setMotor1Pos(0);
                    setMotor2Pos(0);
                    setMotor1Pos(u_low);
                    setMotor2Pos(u_high);
                }
            }
            break;
        case 3158572032: // red down arrow
            if (abs(y) >= deadBand2)
            {
                if(T1_avg >= T2_avg)
                {
                    u_high = 895 + abs(y);
                    u_low = 860 - abs(y);
                    sprintf(line,"u_high: %" PRIi32 " u_low: %" PRIi32 "\n", u_high, u_low );
                    putsUart0(line);
                    sprintf(line,"T1_avg: %f  T2_avg  %f\n", T1_avg, T2_avg);
                    putsUart0(line);
                    putsUart0("\n");
                    setMotor1Neg(u_high);
                    setMotor2Neg(u_low);
                }

                if(T2_avg > T1_avg)
                {
                    u_high = 860 + abs(y);
                    u_low = 895 - abs(y);
                    sprintf(line,"u_high: %" PRIi32 " u_low: %" PRIi32 "\n", u_high, u_low );
                    putsUart0(line);
                    sprintf(line,"T1_avg: %f  T2_avg  %f\n", T1_avg, T2_avg);
                    putsUart0(line);
                    putsUart0("\n");
                    setMotor1Neg(u_low);
                    setMotor2Neg(u_high);
                }
            }
            //setMotor1Neg(830);
            //setMotor2Neg(830);
            signal = 0;
            break;
        case 4244896768:
            putsUart0("TURN OFF MOTORS\n");
            setMotor1Pos(0);
            setMotor2Pos(0);
            setMotor1Neg(0);
            setMotor2Neg(0);
            signal = 0;
            break;
        case 2623798272:
            putsUart0("Turn\n");
            setMotor1Pos(895);
            setMotor2Neg(860);
            waitMicrosecond(230000);
            setMotor1Pos(0);
            setMotor2Neg(0);
            signal = 0;
            break;
        case 2556951552:
            putsUart0("Turn other\n");
            setMotor1Neg(895);
            setMotor2Pos(860);
            waitMicrosecond(230000);
            setMotor1Neg(0);
            setMotor2Pos(0);
            signal = 0;
            break;
        case 2373188608: //RESET
            putsUart0("RESET\n");
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            signal = 0;
            break;
        default:
            break;
        }
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

void wideTimer1Isr() //motor 1 optical encoder reader
{
    time1 = WTIMER1_TAV_R;                        // read counter input
    WTIMER1_TAV_R = 0;                           // zero counter for next edge

    sum1 -= time1Array[time1Index];
    sum1 += time1;
    time1Array[time1Index] = time1;
    time1Index = (time1Index + 1) % 39;
    T1_avg_bits = (sum1/40);
    T1_avg = T1_avg_bits/40000;

    /*
    sprintf(line,"Index: %" PRIu32 "\n", time1Index );
    putsUart0(line);
    sprintf(line,"time 1: %" PRIu32 "\n", time1Array[time1Index] );
    putsUart0(line);
    sprintf(line,"T1_avg_bits: %" PRIu32 "\n", T1_avg_bits );
    putsUart0(line);
    sprintf(line,"T1_avg: %f \n", T1_avg );
    putsUart0(line);
    sprintf(line,"error: %" PRIi32 "\n", error );
    putsUart0(line);
    sprintf(line,"u_high %" PRIu32 " \n", u_high );
    putsUart0(line);
    sprintf(line,"u_low %" PRIu32 " \n\n", u_low );
    putsUart0(line);
    */

    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void wideTimer5Isr() // motor 2 optical encoder reader
{
    time2 = WTIMER5_TAV_R;                        // read counter input
    WTIMER5_TAV_R = 0;                           // zero counter for next edge

    sum2 -= time2Array[time2Index];
    sum2 += time2;
    time2Array[time2Index] = time2;
    time2Index = (time2Index + 1) % 39;
    T2_avg_bits = (sum2/40);
    T2_avg = T2_avg_bits/40000;

    //sprintf(line,"time 2: %" PRIu32 "\n", time2Array[time2Index] );
    //putsUart0(line);
    WTIMER5_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void initHw()
{
    initSystemClockTo40Mhz();
    enablePort(PORTB); //has 2 PWMs
    enablePort(PORTC); //has 2 PWMs has a WT
    enablePort(PORTD); //has timer for IR sensor, has a WT
    enablePort(PORTE); //has 1 GPIO
    enablePort(PORTF); //2 time bases

    selectPinPushPullOutput(PORTC,4);
    selectPinPushPullOutput(PORTC,5);
    selectPinPushPullOutput(PORTB,6);
    selectPinPushPullOutput(PORTB,7);

    //SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2 | SYSCTL_RCGCTIMER_R4;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R3 | SYSCTL_RCGCWTIMER_R5;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    _delay_cycles(3);

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTD_AFSEL_R |= FREQ_IN_MASK;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD2_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD2_WT3CCP0;
    GPIO_PORTD_DEN_R |= FREQ_IN_MASK;

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTC_AFSEL_R |= 64;                        // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= 64;                          // enable bit 6 for digital input

    // Configure SIGNAL_IN for frequency and time measurements
    GPIO_PORTD_AFSEL_R |= 64;                        // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD6_WT5CCP0;
    GPIO_PORTD_DEN_R |= 64;                          // enable bit 6 for digital input

    //----------------------------INITIALIZE MOTORS----------------------------------------------------------------

    setPinAuxFunction(PORTC, 4, GPIO_PCTL_PC4_M0PWM6);
    setPinAuxFunction(PORTC, 5, GPIO_PCTL_PC5_M0PWM7);
    setPinAuxFunction(PORTB, 7, GPIO_PCTL_PB7_M0PWM1);
    setPinAuxFunction(PORTB, 6, GPIO_PCTL_PB6_M0PWM0);

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_0_CTL_R = 0;                                // turn-off PWM0 generator 0 (drives outs 0 and 1)
    PWM0_3_CTL_R = 0;                                // turn-off PWM0 generator 3 (drives outs 6 and 7)
    PWM0_0_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;
    PWM0_3_GENA_R = PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;

    PWM0_0_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

    PWM0_0_CMPB_R = 0;
    PWM0_0_CMPA_R = 0;
    PWM0_3_CMPB_R = 0;
    PWM0_3_CMPA_R = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM0_ENABLE_R = PWM_ENABLE_PWM0EN | PWM_ENABLE_PWM1EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
}

int main(void)
{
    initHw();
    initUart0();
    initI2c0();
    setUart0BaudRate(115200, 40e6);
    enableTimers();

    //0b01101000 = 0x68
    //useful datasheet pages: 33, 36 on product guide
    //writeI2c0Register(0x68, 0x38, 0x01); //Interrupt Configuration
    writeI2c0Register(0x68, 0x19, 0x07); //Sample rate divider Register
    writeI2c0Register(0x68, 0x6B, 0x02); //Power Management Register
    writeI2c0Register(0x68, 0x1A, 0x00); //Configuration Register: FSYNC AND DLPF
    writeI2c0Register(0x68, 0x1B, 0x08); //Gyroscope Configuration Register
    writeI2c0Register(0x68, 0x1C, 0x10); //Accelerometer Configuration Register + DHPF

    calibrateGyro();
    initPidControllers();
    while(true)
    {
//        putsUart0("\n");
//        waitMicrosecond(250000);
    }

    while(true);
	return 0;
}

/*
data[1] = readI2c0Register(0x68, 0x3B);  //ACCEL X MSB
data[2] = readI2c0Register(0x68, 0x3C);  //ACCEL X LSB
data[3] = readI2c0Register(0x68, 0x3D);  //ACCEL Y MSB
data[4] = readI2c0Register(0x68, 0x3E);  //ACCEL Y LSB
data[5] = readI2c0Register(0x68, 0x3F);  //ACCEL Z MSB
data[6] = readI2c0Register(0x68, 0x40);  //ACCEL Z LSB
data[7] = readI2c0Register(0x68, 0x43);  //GYRO X MSB
data[8] = readI2c0Register(0x68, 0x44);  //GYRO X LSB
data[9] = readI2c0Register(0x68, 0x45);  //GYRO Y MSB
data[10] = readI2c0Register(0x68, 0x46); //GYRO Y LSB
data[11] = readI2c0Register(0x68, 0x47); //GYRO Z MSB
data[12] = readI2c0Register(0x68, 0x48); //GYRO Z LSB
*/

