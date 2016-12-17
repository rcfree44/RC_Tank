/*
 * ServoCmd
 */

#include "ServoCmd.h"

#if defined(__AVR_ATmega2560__)
  #define int0 (PINE & (1<<4)) // Faster than digitalRead(2) 
  #define int1 (PINE & (1<<5)) // Faster than digitalRead(3)
#else // elif defined()
  #define int0 (PIND & (1<<2)) // Faster than digitalRead(2)
  #define int1 (PIND & (1<<3)) // Faster than digitalRead(3)
  #define int2 (PINB & (1<<0)) // Faster than digitalRead(8)
#endif

#define READ_SERVO_NB 3

volatile U16 ReadServo_Pulse[READ_SERVO_NB];
volatile U16 ReadServo_Base[READ_SERVO_NB];

static uint16_t fast_us()
{
  uint16_t _us = TCNT1;
  if (_us) return _us;
  return 1; // avoid null value
}

void handleInterrupt_Ch0() 
{ 
  const uint16_t _us = fast_us();
  if(int0) 
  {
    ReadServo_Base[0] = _us; // we got a positive edge 
  }
  else if (ReadServo_Base[0])
  {
    ReadServo_Pulse[0] = _us - ReadServo_Base[0]; // Negative edge: get pulsewidth
    ReadServo_Base[0]  = 0;
  }
}

void handleInterrupt_Ch1() 
{ 
  const uint16_t _us = fast_us();
  if(int1) 
  {
    ReadServo_Base[1] = _us; // we got a positive edge 
  }
  else if (ReadServo_Base[1])
  {
    ReadServo_Pulse[1] = _us - ReadServo_Base[1]; // Negative edge: get pulsewidth
    ReadServo_Base[1]  = 0;
  }
} 

#if READ_SERVO_NB > 2
ISR(PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
  const uint16_t _us = fast_us();
  if(int2) 
  {
    ReadServo_Base[2] = _us; // we got a positive edge 
  }
  else if (ReadServo_Base[2])
  {
    ReadServo_Pulse[2] = _us - ReadServo_Base[2]; // Negative edge: get pulsewidth
    ReadServo_Base[2]  = 0;
  }
}
#endif // READ_SERVO_NB

void  ServoCmd_InputInit()
{
  // reset
  memset((void *)ReadServo_Base,  0, sizeof(ReadServo_Base));
  memset((void *)ReadServo_Pulse, 0, sizeof(ReadServo_Pulse));
  
  // Fast Timer 1
  {
    TCCR1A = 0;             // normal counting mode 
    TCCR1B = _BV(CS11);     // set prescaler of 8 
    TCNT1  = 0;             // clear the timer count 
  }

  // IT Change
  {
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    attachInterrupt(0, handleInterrupt_Ch0, CHANGE);  
    attachInterrupt(1, handleInterrupt_Ch1, CHANGE);
  }
  
  // Pin change interrupt for D8 to D13 here
#if READ_SERVO_NB > 2
  {
    *digitalPinToPCMSK(8) |= bit(digitalPinToPCMSKbit(8));  // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(8)); // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(8)); // enable interrupt for the group
  }
#endif // READ_SERVO_NB
}

U16  ServoCmd_InputRead(U8 Channel, bool ClearAfterRead /*= false*/)
{
  if (Channel < READ_SERVO_NB)
  {
    // IT Safe reading
    U16 Val = ReadServo_Pulse[Channel];
    while (ReadServo_Pulse[Channel] != Val)
    {
      Val = ReadServo_Pulse[Channel];
    }
    // IT Safe clearing
    if (ClearAfterRead)
    {
      while (ReadServo_Pulse[Channel] != 0)
      {
        ReadServo_Pulse[Channel] = 0;
      }
    }
    // Apply 16Mhz TCNT1 rescaling 
  #if F_CPU == 16000000
    Val >>= 1;
  #endif
    if (Val >= PWM_MIN_LIMIT && Val <= PWM_MAX_LIMIT)
    {
      return Val;
    }
  }
  return 0;
}
