#ifndef COMMON_H
#define COMMON_H

#include <FlexiTimer2.h>
typedef unsigned char  u8, U8, uint8, UINT8, BYTE;
typedef signed char   s8, S8, int8, INT8;

typedef unsigned short  u16, U16, uint16, UINT16, WORD;
typedef signed short  s16, S16, int16, INT16;

typedef unsigned long u32, U32, uint32, UINT32, DWORD;
typedef signed long   s32, S32, int32, INT32;

#define BIT(n) (1<<(n))

#define     BYTE0(n)            ((unsigned char)((unsigned short)(n)))
#define     BYTE1(n)            ((unsigned char)(((unsigned short)(n))>>8))
#define     BYTE2(n)            ((unsigned char)(((unsigned short)(((unsigned long)(n))>>8))>>8))
#define     BYTE3(n)            ((unsigned char)(((unsigned short)(((unsigned long)(n))>>16))>>8))

#define TRUE   1
#define FALSE  0
#define true   1
#define false  0

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#endif
