// QEI0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// Quadrature Encoder:
//   PhA0 (PD6) and PhB0 (PD7) are connected to 256 cpr optical encoder

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "qei0.h"
#include "gpio.h"

// PortD masks
#define PHA0 PORTD,6
#define PHB0 PORTD,7

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initQei0()
{
    // Enable clocks
    SYSCTL_RCGCQEI_R |= SYSCTL_RCGCQEI_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);

    // Configure PhA0 and PhB0 for QE
    setPinCommitControl(PHB0);
    selectPinDigitalInput(PHA0);
    selectPinDigitalInput(PHB0);
    setPinAuxFunction(PHA0, GPIO_PCTL_PD6_PHA0);
    setPinAuxFunction(PHB0, GPIO_PCTL_PD7_PHB0);

    // Configure QE
    QEI0_CTL_R = QEI_CTL_CAPMODE; // capture on both edges
    QEI0_MAXPOS_R = 0xFFFFFFFF;   // chosen for int32_t return value
    QEI0_CTL_R |= QEI_CTL_ENABLE;
}

// Set position
void setQei0Position(int32_t  pos)
{
    QEI0_POS_R = pos;
}

// Get position
int32_t getQei0Position()
{
    return QEI0_POS_R;
}
