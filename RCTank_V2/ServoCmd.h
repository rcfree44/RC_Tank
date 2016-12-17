/*
 * ServoCmd_Input
 */
 
#ifndef ServoCmd_h
#define ServoCmd_h

#include "LIBcommon.h"

// Constants
#define PWM_MIN_LIMIT   400
#define PWM_MIN_VAL     540
#define PWM_NOM_VAL    1520 
#define PWM_MAX_VAL    2400
#define PWM_MAX_LIMIT  2600

void ServoCmd_InputInit();
U16  ServoCmd_InputRead(U8 Channel, bool ClearAfterRead = false);

#endif // ServoCmd_h
