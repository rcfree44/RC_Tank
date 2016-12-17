/*
 * LIBCommon
 */

#ifndef LIBcommon_h
#define LIBcommon_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

// Workaround for http://gcc.gnu.org/bugzilla/show_bug.cgi?id=34734
#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

// Data Types
#define U1_LIMIT  0x1
#define U2_LIMIT  0x3
#define U4_LIMIT  0xF
#define U8_LIMIT  0xFF
#define U16_LIMIT 0xFFFF
#define U32_LIMIT 0xFFFFFFFF
#define S8_LIMIT  0x7F
#define S16_LIMIT 0x7FFF
#define S32_LIMIT 0x7FFFFFFF
typedef unsigned char  U8;
typedef unsigned short U16;
typedef unsigned long  U32;
typedef signed char    S8;
typedef signed short   S16;
typedef signed long    S32;

// IO Mapping
#define IO_OUT_SERIAL     0
#define IO_IN_SERIAL      1
#define IO_IN_CHANNEL1    2
#define IO_OUT_LIGHT1     3
#define IO_IN_CHANNEL2    4
#define IO_OUT_LIGHT2     5
#define IO_OUT_DEBUG      5
#define IO_OUT_LIGHT3     6
#define IO_IN_CHANNEL3    7
#define IO_IN_CHANNEL4    8
#define IO_OUT_LIGHT4     9
#define IO_OUT_SPI_DATA   10
#define IO_OUT_SPI_LOAD   11
#define IO_OUT_SPI_CLK    12
#define IO_OUT_STATUS     13

#define IO_ANA_VIN        A0
#define IO_ANA_KEY        A1
#define IO_ANA_SDA        A4
#define IO_ANA_SCL        A5

// Basic Math
U16 U16_Min(U16 Val1, U16 Val2); 
U16 U16_Max(U16 Val1, U16 Val2); 
U16 U16_Cap(U16 Val, U16 Min, U16 Max); 
S16 S16_Cap(S16 Val, S16 Min, S16 Max); 

// String Management
extern void PutDecNumber2(char *ptr, U8 val);
extern void PutDecNumber3(char *ptr, U16 val);
extern void PutHexNumber8(char *ptr, U8 val);
extern void PutHexNumber16(char *ptr, U16 val);
extern void PutHexNumber32(char *ptr, U32 val);
extern U8   GetHexNumber8(char *ptr);
extern U16  GetHexNumber16(char *ptr);
extern U32  GetHexNumber32(char *ptr);

// Serial Management
extern bool SerialWaitInput(U32 TimeOutMs);
extern int  SerialReadNumber(U32 TimeOutMs);

// CRC
extern U16 CHKUpdate(U16 CHK, U8 * Data, U16 Size);

// E2P Read & Write (Buffer)
extern U16 E2PWrite(U16 BaseAddr, U8 * Data, U16 Size);
extern U16 E2PRead (U16 BaseAddr, U8 * Data, U16 Size);

// E2P Read & Write (Size + CRC)
extern bool E2PWrite_SizeCRC(U16 BaseAddr, U8 * Data, U16 Size);
extern bool E2PRead_SizeCRC(U16 BaseAddr, U8 * Data, U16 Size);


#endif // LIBcommon_h
