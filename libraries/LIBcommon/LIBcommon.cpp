/*
 * LIBCommon
 */

#include "LIBcommon.h"
#include <EEPROM.h>

// Basic Math
U16 U16_Min(U16 Val1, U16 Val2)
{
  if (Val1 < Val2) return Val1;
  return Val2;
}
U16 U16_Max(U16 Val1, U16 Val2)
{
  if (Val1 < Val2) return Val2;
  return Val1;
}
U16 U16_Cap(U16 Val, U16 Min, U16 Max)
{
  if (Val < Min) return Min;
  if (Val > Max) return Max;
  return Val;
}
S16 S16_Cap(S16 Val, S16 Min, S16 Max)
{
  if (Val < Min) return Min;
  if (Val > Max) return Max;
  return Val;
} 

// Serial Management
bool SerialWaitInput(U32 TimeOutMs)
{
  U32 BaseTime = millis();
  while((millis() - BaseTime) < TimeOutMs)
  {
    // Wait any kind of data
    if (Serial.read() >= 0)
    {
      // Flush incomming data (during 20 ms)
      BaseTime = millis();
      while((millis() - BaseTime) < 20)
      {
        Serial.read(); // flush
      }
      return true;
    }
  }
  return false;
}

int SerialReadNumber(U32 TimeOutMs)
{
  U32 BaseTime = millis();
  int DigitCount = 0;
  int DigitValue = 0;
  while((millis() - BaseTime) < TimeOutMs)
  {
    int c = Serial.read();
    if (c >= '0' && c <= '9')
    {
      DigitValue *= 10;
      DigitValue += c - '0';
      ++DigitCount;
    }
    else if (c == '.')
    {
      break;
    }
  }
  if (DigitCount > 0)
  {
    return DigitValue;
  }
  return -1;
}

//
static char Hex2Ascii(U8 val)
{
  val &= 0xF;
  return (val >= 10) ? ('A' + (val - 10)) : ('0' + val);
}
static U8 Ascii2Hex(char c)
{
  if (c >= 'A' && c <= 'F')
    return (c - 'A') + 10;
  if (c >= 'a' && c <= 'f')
    return (c - 'a') + 10;
  if (c >= '0' && c <= '9')
    return (c - '0') + 0;
  return 0;
}

//
void PutDecNumber2(char *ptr, U8 val)
{
  *ptr++ = Hex2Ascii(val / 10);
  *ptr   = Hex2Ascii(val % 10);
}
void PutDecNumber3(char *ptr, U16 val)
{
 *ptr++ = Hex2Ascii(val / 100);
 *ptr++ = Hex2Ascii((val % 100)/10);
 *ptr   = Hex2Ascii((val % 100)%10);
}
//
void PutHexNumber8(char *ptr, U8 val)
{
  *ptr++ = Hex2Ascii(val >> 4);
  *ptr   = Hex2Ascii(val >> 0);
}
void PutHexNumber16(char *ptr, U16 val)
{
  *ptr++ = Hex2Ascii(val >> 12);
  *ptr++ = Hex2Ascii(val >> 8);
  *ptr++ = Hex2Ascii(val >> 4);
  *ptr   = Hex2Ascii(val >> 0);
}
void PutHexNumber32(char *ptr, U32 val)
{
  *ptr++ = Hex2Ascii(val >> 28);
  *ptr++ = Hex2Ascii(val >> 24);
  *ptr++ = Hex2Ascii(val >> 20);
  *ptr++ = Hex2Ascii(val >> 16);
  *ptr++ = Hex2Ascii(val >> 12);
  *ptr++ = Hex2Ascii(val >> 8);
  *ptr++ = Hex2Ascii(val >> 4);
  *ptr   = Hex2Ascii(val >> 0);
}
//
U8 GetHexNumber8(char *ptr)
{
  U8 val = 0;
  val |= Ascii2Hex(*ptr++) << 4;
  val |= Ascii2Hex(*ptr  ) << 0;
  return val;
}
U16 GetHexNumber16(char *ptr)
{
  U16 val = 0;
  val |= (U16)Ascii2Hex(*ptr++) << 12;
  val |= (U16)Ascii2Hex(*ptr++) << 8;
  val |= (U16)Ascii2Hex(*ptr++) << 4;
  val |= (U16)Ascii2Hex(*ptr  ) << 0;
  return val;
}
U32 GetHexNumber32(char *ptr)
{
  U32 val = 0;
  val |= (U32)Ascii2Hex(*ptr++) << 28;
  val |= (U32)Ascii2Hex(*ptr++) << 24;
  val |= (U32)Ascii2Hex(*ptr++) << 20;
  val |= (U32)Ascii2Hex(*ptr++) << 16;
  val |= (U32)Ascii2Hex(*ptr++) << 12;
  val |= (U32)Ascii2Hex(*ptr++) << 8;
  val |= (U32)Ascii2Hex(*ptr++) << 4;
  val |= (U32)Ascii2Hex(*ptr  ) << 0;
  return val;
}

// CRC
U16 CHKUpdate(U16 CHK, U8 * Data, U16 Size)
{
  while(Size > 0)
  {
    CHK += 0xFF - (*Data++);
  }
  return CHK;
}

// E2P Read & Write (Buffer)
U16 E2PWrite(U16 BaseAddr, U8 * Data, U16 Size)
{
  while(Size > 0)
  {
    EEPROM.write(BaseAddr++, *Data++);
    --Size;
  }
  return BaseAddr;
}
U16 E2PRead(U16 BaseAddr, U8 * Data, U16 Size)
{
  while(Size > 0)
  {
    *Data++ = EEPROM.read(BaseAddr++);
    --Size;
  }
  return BaseAddr;
}

// E2P Read & Write (Size + CRC)
bool E2PWrite_SizeCRC(U16 BaseAddr, U8 * Data, U16 Size)
{
  BaseAddr = E2PWrite(BaseAddr, (U8*)&Size, sizeof(U16));
  BaseAddr = E2PWrite(BaseAddr, Data, Size);
  const U16 CHK = CHKUpdate(0, Data, Size);
  BaseAddr = E2PWrite(BaseAddr, (U8*)&CHK,  sizeof(U16));
  return true;
}
bool E2PRead_SizeCRC(U16 BaseAddr, U8 * Data, U16 Size)
{
  U16 _Size = 0; 
  U16 _CHK  = 0;
  BaseAddr = E2PRead(BaseAddr, (U8*)&_Size, sizeof(U16));
  if (_Size == Size)
  {
    BaseAddr = E2PRead(BaseAddr, Data, Size);
    const U16 CHK = CHKUpdate(0, Data, Size);
    BaseAddr = E2PRead(BaseAddr, (U8*)&_CHK, sizeof(U16));
    if (_CHK == CHK)
    {
      return true;
    }
  }
  return false;
}
