	/*
 * RCTank_Main
 */
#include "LIBcommon.h"
#include "RCTank_Main.h"
#include "ServoCmd.h"

#define S8_LIMIT  127

// PWM Absolute Limits (min, max)
#define MOTOR_PWM_MAX           255  // will be reduce by Compute_Motor_MaxPWM
#define MOTOR_PWM_MIN             8  // Dead Zone 

#if (MOTOR_PWM_MAX != 255) || (MOTOR_PWM_MAX <= MOTOR_PWM_MIN)
  #error "Config Error"
#endif

// PWM Steering/Thottle power 
#define MOTOR_FWR_POWER_Q8   256 // Forward 
#define MOTOR_REV_POWER_Q8   128 // Reverse
#define MOTOR_STR_POWER_Q8   128 // Steering

#if (MOTOR_FWR_POWER_Q8 > 256) || (MOTOR_REV_POWER_Q8 > MOTOR_FWR_POWER_Q8) || (MOTOR_STR_POWER_Q8 > 256)
  #error "Config Error"
#endif

// Synchro Time
#define SYNCHRO_MS      20 // 50Hz
#define SYNCHRO_PWM     0  // 1: Force PWM Synchro, 0 don't

// IO Config
#define DO_MOTOR_PWM1   5
#define DO_MOTOR_PWM2   6

#define DO_MOTOR_FRW1   4
#define DO_MOTOR_FRW2   7

#define AN_VBAT         A5

//#define DO_DEBUG_1      8
//#define DO_DEBUG_2      9

/* VBAT Reading:

   6.00V: 933   5.10V: 791   4.50V: 698   3.90V: 606   3.20V: 496
   5.80V: 901   5.00V: 778   4.40V: 684   3.80V: 590   3.00V: 465
   5.60V: 873   4.90V: 761   4.30V: 666   3.70V: 574   2.80V: 434
   5.40V: 838   4.80V: 745   4.20V: 650   3.60V: 560   2.60V: 403
   5.30V: 823   4.70V: 729   4.10V: 637   3.50V: 544   2.40V: 372
   5.20V: 806   4.60V: 715   4.00V: 620   3.40V: 527   2.20V: 341

   Average Coef is 155 ADC/V   
*/
#define VBAT_ADC_ALARM_MAX    900 // ~5.8V
#define VBAT_ADC_ALARM_MIN    620 // ~4.0V
//////////////////////////////////////////
#define VBAT_ADC_PWM_MAX      500 // ~3.2V

U8 Compute_Motor_MaxPWM(void)
{
  // Read VBat battery
  const U16 VbatAdc = analogRead(AN_VBAT);
  
  // Check Min/Max
  if ((VbatAdc <= VBAT_ADC_ALARM_MIN) || (VbatAdc >= VBAT_ADC_ALARM_MAX))
  {
    // Power OFF
    return 0;
  }
  
  // Check Power
  if (VbatAdc <= VBAT_ADC_PWM_MAX)
  {
    return MOTOR_PWM_MAX;
  }
  
  // Reduce Power
  return (((U32)MOTOR_PWM_MAX * (U32)VBAT_ADC_PWM_MAX) + (VbatAdc/2))/VbatAdc;
}

/* DX3c Limits
  Steering : -150% :  928  Throttle : -150% :  944 
  Steering : -125% : 1008  Throttle : -125% : 1024
  Steering : -100% : 1104  Throttle : -100% : 1128
  Steering : - 75% : 1208  Throttle : - 75% : 1224
  Steering : - 50% : 1312  Throttle : - 50% : 1328
  Steering :    0% : 1504  Throttle :    0% : 1480
  Steering : + 50% : 1712  Throttle : + 50% : 1736
  Steering : + 75% : 1816  Throttle : + 75% : 1840
  Steering : +100% : 1920  Throttle : +100% : 1936
  Steering : +125% : 2016  Throttle : +125% : 2040
  Steering : +150% : 2096  Throttle : +150% : 2120

  NOTA1: From real measurement the Steering value is very sharp (1µs error <=> not measurable error)

  NOTA2: From real measurement the Throttle value is over-counter by about 16 µs,  
         due to synchronized signal-falling (Steering IT treatment takes some time).

         This cause is little non-linearility :
          - 1.72% error at -150% 
          - 0.76% error at +150%
          => Less than 1% non-linearility
         AND not very important for the Trottle response.
*/

typedef struct
{
  // Config
  U16 Min;      // about 1000 µs
  U16 Center;   // about 1500 µs
  U16 Max;      // about 2000 µs
  
  // Pre-Computation
  U16 SpanMin_d2;
  U16 SpanMax_d2;
  
} Ch_Timing_t;

// System
bool Ch_Valid;
U16  Ch_Synchro;

// Motor Command
U8 Motor_PWM1, Motor_Old_PWM1;
U8 Motor_PWM2, Motor_Old_PWM2; 
bool Motor_FRW1, Motor_FRW2;

// Servo Inputs
Ch_Timing_t Ch1_Timing = {1105, 1504, 1914, 0, 0}; // My DX3C + a Orange GR300 DSM2 Receiver (no automatic learning)
Ch_Timing_t Ch2_Timing = {1120, 1468, 1930, 0, 0}; // My DX3C + a Orange GR300 DSM2 Receiver (no automatic learning)
Ch_Timing_t Ch3_Timing = {1135, 1540, 1944, 0, 0}; // My DX3C + a Orange GR300 DSM2 Receiver (no automatic learning)

bool Ch_Timing_Set(Ch_Timing_t & t)
{
  bool Status = false;

  // Compute
  if ((t.Min < t.Max) && (t.Center > t.Min) && (t.Center < t.Max))
  {
    // Max Span/2 is 1000
    if (t.Min >= 500 && t.Max <= 2500)
    {
      // Span is divized by 2 and majorated to avoid S8_LIMIT overflow
      t.SpanMin_d2 = 1+(((t.Center - t.Min)+1)/2); 
      t.SpanMax_d2 = 1+(((t.Max - t.Center)+1)/2);
      // Max Span/2 is 1000 (501 = 1000/2 + 1)
      if ((t.SpanMin_d2 > 0 && t.SpanMin_d2 <= 501) && 
          (t.SpanMax_d2 > 0 && t.SpanMax_d2 <= 501)   )
      {
        Status = true;
      }
    }
  }

  // Print
#if 0
  Serial.println(F("--------------------------"));
  Serial.print  (F("Channel:    ")); Serial.println(Print_Name);
  Serial.print  (F("Min:        ")); Serial.println(t.Min );
  Serial.print  (F("Center:     ")); Serial.println(t.Center);
  Serial.print  (F("Max:        ")); Serial.println(t.Max);
  Serial.print  (F("SpanMin_d2: ")); Serial.println(t.SpanMin_d2);
  Serial.print  (F("SpanMax_d2: ")); Serial.println(t.SpanMax_d2);
  Serial.print  (F("Status:     ")); Serial.println(Status);
#endif
  return Status;
}

S8 Convert_Timing(U16 Input, Ch_Timing_t & t)
{
  if (Input == 0)
  {
    // Zero (or Error)
    return 0;
  }
  else if (Input <= t.Min)
  {
    // Min Min
    return -S8_LIMIT;
  }
  else if (Input >= t.Max)
  {
    // Max Max
    return +S8_LIMIT;
  }
  else if (Input >= t.Center)
  {
    // Max: SpanMax is divized by 2 and majorated to avoid +S8_LIMIT overflow
    return +(((64*(Input - t.Center))+(t.SpanMax_d2/2))/t.SpanMax_d2);
  }
  else
  {
    // Min: SpanMin is divized by 2 and majorated to avoid -S8_LIMIT overflow
    return -(((64*(t.Center - Input))+(t.SpanMin_d2/2))/t.SpanMin_d2);
  }
}

bool RCTank_Main_Init()
{
  // Channel Set & Validate
  Ch_Valid = true;
  if (true != Ch_Timing_Set(Ch1_Timing))
  {
    Ch_Valid = false;
  }
  if (true != Ch_Timing_Set(Ch2_Timing))
  {
    Ch_Valid = false;
  }
  if (true != Ch_Timing_Set(Ch3_Timing))
  {
    Ch_Valid = false;
  }

  // Motor Reset
  Motor_PWM1 = Motor_Old_PWM1 = 0;
  Motor_PWM2 = Motor_Old_PWM2 = 0;
  Motor_FRW1 = Motor_FRW2 = false;

  // Enable Power
  if (Ch_Valid)
  {
    pinMode(DO_MOTOR_PWM1, OUTPUT); analogWrite(DO_MOTOR_PWM1, Motor_PWM1);
    pinMode(DO_MOTOR_PWM2, OUTPUT); analogWrite(DO_MOTOR_PWM2, Motor_PWM2);
    pinMode(DO_MOTOR_FRW1, OUTPUT); digitalWrite(DO_MOTOR_FRW1, Motor_FRW1);
    pinMode(DO_MOTOR_FRW2, OUTPUT); digitalWrite(DO_MOTOR_FRW2, Motor_FRW2);
  #ifdef DO_DEBUG_1
    pinMode(DO_DEBUG_1, OUTPUT);
  #endif
  #ifdef DO_DEBUG_2
    pinMode(DO_DEBUG_2, OUTPUT);
  #endif
  }
  
  // Synchro
  Ch_Synchro = (U16)millis();
  
  // Return
  return Ch_Valid;
}

//U16 Cnt_Loop;
//U16 Cnt_Err;

bool RCTank_Main_Loop()
{
  bool Status = false;
  
#ifdef DO_DEBUG_1
  digitalWrite(DO_DEBUG_1, HIGH);
#endif

  // Reset Motor Power
  Motor_PWM1 = Motor_PWM2 = 0;
  Motor_FRW1 = Motor_FRW2 = false;
  
  // Compute the Max Motor Power (depends of the max charge voltage & current battery level)
  const U8 Motor_MaxPWM = Compute_Motor_MaxPWM();  
  
#if 1
  // Update if Valid & MaxPower computed
  if ((Ch_Valid) && (Motor_MaxPWM > 0))
  {
    // Get Next Value : normally 16.44ms loop (60.87Hz) 
  #if SYNCHRO_PWM == 1
  #ifdef DO_DEBUG_2
    digitalWrite(DO_DEBUG_2, HIGH);
  #endif
    // Synchro PWM
    U16 Ch1_Raw = 0;
    U16 Ch2_Raw = 0;
    U16 Ch3_Raw = 0;
    U16 TimeOut = (U16)millis();
    do
    {
      // Read Radio Inputs
      if (0 == Ch1_Raw) Ch1_Raw = ServoCmd_InputRead(0, true/*clear after read*/);
      if (0 == Ch2_Raw) Ch2_Raw = ServoCmd_InputRead(1, true/*clear after read*/);
      if (0 == Ch3_Raw) Ch3_Raw = ServoCmd_InputRead(2, true/*clear after read*/);

      // TimeOut
      if (((U16)millis() - TimeOut) > 30)
      {
        Ch1_Raw = Ch2_Raw = Ch3_Raw = 0;
        break;
      }
    }
    while ((0 == Ch1_Raw) || (0 == Ch2_Raw) || (0 == Ch3_Raw)); // Wait for all stable signals (TimeOut guarded)
  #ifdef DO_DEBUG_2
    digitalWrite(DO_DEBUG_2, LOW);
  #endif
  #else
    // Get latest PWM (un-synchronized read)
    const U16 Ch1_Raw = ServoCmd_InputRead(0);
    const U16 Ch2_Raw = ServoCmd_InputRead(1);
    const U16 Ch3_Raw = ServoCmd_InputRead(2);
  #endif

    // Status Management
    if ((0 == Ch1_Raw) || (0 == Ch2_Raw) || (0 == Ch3_Raw))
    {
      // ++Cnt_Err;

      // Re-use old command (dangerous ?)
      // Motor_PWM1 = Motor_Old_PWM1;
      // Motor_PWM2 = Motor_Old_PWM2;
    }
    else
    {  
      // Channel S8
      S8 Ch1_S8 = 0;
      S8 Ch2_S8 = 0;
      S8 Ch3_S8 = 0;
      {
        Ch1_S8 = Convert_Timing(Ch1_Raw, Ch1_Timing); // Steering
        Ch2_S8 = Convert_Timing(Ch2_Raw, Ch2_Timing); // Throttle
        Ch3_S8 = Convert_Timing(Ch3_Raw, Ch3_Timing); // Throttle
      
      #if 0
        Serial.print("Ch1 S8: "); Serial.print(Ch1_S8); Serial.print(" ("); Serial.print(Ch1_Raw); Serial.println(")");  
        Serial.print("Ch2 S8: "); Serial.print(Ch2_S8); Serial.print(" ("); Serial.print(Ch2_Raw); Serial.println(")"); 
        Serial.print("Ch3 S8: "); Serial.print(Ch3_S8); Serial.print(" ("); Serial.print(Ch3_Raw); Serial.println(")"); 
      #endif
      }
      
      // Motor Control
      S16 Mt1_S8 = 0;
      S16 Mt2_S8 = 0;
      {
        // Apply Throttle
        if (Ch2_S8 > 0) 
        {
          const U8 Throttle_S8 = (((U16)Ch2_S8 * (U16)MOTOR_FWR_POWER_Q8)+128)/256;
          Mt1_S8 += Throttle_S8;
          Mt2_S8 += Throttle_S8;
        }
        else if (Ch2_S8 < 0) 
        {
          const U8 Throttle_S8 = (((U16)(-Ch2_S8) * (U16)MOTOR_REV_POWER_Q8)+128)/256;
          Mt1_S8 -= Throttle_S8;
          Mt2_S8 -= Throttle_S8;
        }
          
        // Apply Steering
        if (Ch1_S8 > 0)
        {
          const U8 Steering_S8 = (((U16)Ch1_S8 * (U16)MOTOR_STR_POWER_Q8)+128)/256;
          Mt1_S8 += Steering_S8;
          Mt2_S8 -= Steering_S8;
        }
        else if (Ch1_S8 < 0)
        {
          const U8 Steering_S8 = (((U16)(-Ch1_S8) * (U16)MOTOR_STR_POWER_Q8)+128)/256;
          Mt1_S8 -= Steering_S8;
          Mt2_S8 += Steering_S8;
        }
        
        // Limit Motor S8
        Mt1_S8 = S16_Cap(Mt1_S8, -S8_LIMIT, +S8_LIMIT);
        Mt2_S8 = S16_Cap(Mt2_S8, -S8_LIMIT, +S8_LIMIT);
        
      #if 0
        Serial.print("Mt1 S8: "); Serial.println(Mt1_S8);
        Serial.print("Mt2 S8: "); Serial.println(Mt2_S8);
      #endif
      }
      
      // PWM & FRW
      {
        // Convert S8 into PWM (Power Limiter)
        if (Mt1_S8 > 0)
        {
          Motor_PWM1 = 1 + ((((U16)Mt1_S8 * (U16)Motor_MaxPWM)+64)/128); // +1 due to 128 division (instead of 127)
          Motor_FRW1 = true;
        }
        else if (Mt1_S8 < 0)
        {
          Motor_PWM1 = 1 + ((((U16)(-Mt1_S8) * (U16)Motor_MaxPWM)+64)/128); // +1 due to 128 division (instead of 127)
          Motor_FRW1 = false;
        }
        
        // Convert S8 into PWM (Power Limiter)
        if (Mt2_S8 > 0)
        {
          Motor_PWM2 = 1 + ((((U16)Mt2_S8 * (U16)Motor_MaxPWM)+64)/128); // +1 due to 128 division (instead of 127)
          Motor_FRW2 = true;
        }
        else if (Mt2_S8 < 0)
        {
          Motor_PWM2 = 1 + ((((U16)(-Mt2_S8) * (U16)Motor_MaxPWM)+64)/128); // +1 due to 128 division (instead of 127)
          Motor_FRW2 = false;
        }
        
        // Apply Dead Zone (limit the low-level glitch)
        if (Motor_PWM1 < MOTOR_PWM_MIN)
        {
          Motor_PWM1 = 0;
        }
        if (Motor_PWM2 < MOTOR_PWM_MIN)
        {
          Motor_PWM2 = 0;
        }

      #if 0
        Serial.print("S.PWM1: "); if (Motor_FRW1) Serial.println((int)Motor_PWM1); else Serial.println((int)-Motor_PWM1);
        Serial.print("S.PWM2: "); if (Motor_FRW2) Serial.println((int)Motor_PWM2); else Serial.println((int)-Motor_PWM2);
      #endif
      }

      // Done
      Status = true;
    }
  }

  // Apply Power (if change)
  if (Motor_Old_PWM1 != Motor_PWM1)
  {
    Motor_Old_PWM1 = Motor_PWM1; 
    analogWrite(DO_MOTOR_PWM1, Motor_PWM1);
  }
  if (Motor_Old_PWM2 != Motor_PWM2)
  {
    Motor_Old_PWM2 = Motor_PWM2; 
    analogWrite(DO_MOTOR_PWM2, Motor_PWM2);
  }
  
  // Apply Forward Command
  digitalWrite(DO_MOTOR_FRW1, Motor_FRW1);
  digitalWrite(DO_MOTOR_FRW2, Motor_FRW2);

#endif

  // Debug
#if 0
  Serial.print("VBAT: "); Serial.println(analogRead(AN_VBAT));  
  Serial.print("MaxPower: "); Serial.println(Motor_MaxPWM); 
  Serial.println("-----------------------------");
  delay(500);
#endif

#if 0
  if (++Cnt_Loop >= 250)
  {
    Serial.print("Err: "); Serial.println(Cnt_Err);
    Cnt_Err = Cnt_Loop = 0;
  }
#endif

  // LED Off
  digitalWrite(LED_BUILTIN, LOW);

  // Wait Synchro
#ifdef DO_DEBUG_1
  digitalWrite(DO_DEBUG_1, LOW);
#endif
#ifdef DO_DEBUG_2
  digitalWrite(DO_DEBUG_2, HIGH);
#endif
  Ch_Synchro += SYNCHRO_MS;
  while (((U16)millis() - Ch_Synchro) < SYNCHRO_MS)
  {
    // do others stuff like NeoPixel lighting
  }
#ifdef DO_DEBUG_2
  digitalWrite(DO_DEBUG_2, LOW);
#endif

  // Return Status
  return Status;
}
