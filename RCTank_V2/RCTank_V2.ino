#include "LIBcommon.h"
#include "ServoCmd.h"
#include "RCTank_Main.h"
#include <EEPROM.h>

void setup()
{
  // Init & Test
  Serial.begin(9600);
  randomSeed(analogRead(0));
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Start();
  ServoCmd_InputInit();
  if (RCTank_Main_Init())
  {
    Serial.println(F("Ready."));
  }
}

void loop() 
{
  if (RCTank_Main_Loop())
  {
    // Short Flash
    digitalWrite(LED_BUILTIN, HIGH);
  }
  // Long Delay
  for(U8 i=0; i<15; ++i)
  {
    RCTank_Main_Loop();
  }
}
