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

#ifndef QEI0_H_
#define QEI0_H_

#include "qei0.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initQei0();
// Set position
void setQei0Position(int32_t  pos);
int32_t getQei0Position();

#endif
