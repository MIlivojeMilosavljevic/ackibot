//
//#ifndef TYPE_SHORTS_H
//#define TYPE_SHORTS_H
//
/////////////////////////////////////////////////////////////////////////////////
//
//#include <stdint.h>
//
/////////////////////////////////////////////////////////////////////////////////
//
//#ifndef __USBAPI__
//typedef uint8_t u8;
//typedef uint16_t u16;
//typedef uint32_t u32;
//typedef uint64_t u64;
//#endif
//
//typedef int8_t i8;
//typedef int16_t i16;
//typedef int32_t i32;
//typedef int64_t i64;
//
/////////////////////////////////////////////////////////////////////////////////
//
//#endif // TYPE_SHORTS_H

#ifndef TYPE_SHORTS_H
#define TYPE_SHORTS_H

///////////////////////////////////////////////////////////////////////////////

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// Koristimo vlastite nazive tipova samo ako vec nisu definisani
// Arduino core već definiše u8, u16, u32, pa da ne dođe do konflikta

#ifndef MY_U8_DEFINED
typedef uint8_t  u8;
#define MY_U8_DEFINED
#endif

//#ifndef MY_U16_DEFINED
//typedef uint16_t u16;
//#define MY_U16_DEFINED
//#endif

#ifndef MY_U32_DEFINED
typedef uint32_t u32;
#define MY_U32_DEFINED
#endif

#ifndef MY_U64_DEFINED
typedef uint64_t u64;
#define MY_U64_DEFINED
#endif

#ifndef MY_I8_DEFINED
typedef int8_t   i8;
#define MY_I8_DEFINED
#endif

#ifndef MY_I16_DEFINED
typedef int16_t  i16;
#define MY_I16_DEFINED
#endif

#ifndef MY_I32_DEFINED
typedef int32_t  i32;
#define MY_I32_DEFINED
#endif

#ifndef MY_I64_DEFINED
typedef int64_t  i64;
#define MY_I64_DEFINED
#endif

///////////////////////////////////////////////////////////////////////////////

#endif // TYPE_SHORTS_H
