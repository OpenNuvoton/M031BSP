#ifndef __PROCESSORS_COMPILER_H
#define __PROCESSORS_COMPILER_H

// Arm Compiler 4/5
#if defined ( __CC_ARM )
#ifndef   __ALIGNED
#define __ALIGNED(x)        __attribute__((aligned(x)))
#endif
#ifndef   __PACKED
#define __PACKED            __attribute__((packed))
#endif
#ifndef   __PACKED_STRUCT
#define __PACKED_STRUCT     __packed struct
#endif
#ifndef   __PACKED_UNION
#define __PACKED_UNION      __packed union
#endif


// Arm Compiler 6 (armclang)
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#ifndef   __ALIGNED
#define __ALIGNED(x)        __attribute__((aligned(x)))
#endif
#ifndef   __PACKED
#define __PACKED            __attribute__((packed, aligned(1)))
#endif
#ifndef   __PACKED_STRUCT
#define __PACKED_STRUCT     struct __attribute__((packed, aligned(1)))
#endif
#ifndef   __PACKED_UNION
#define __PACKED_UNION      union __attribute__((packed, aligned(1)))
#endif


// GNU Compiler
#elif defined ( __GNUC__ )
#ifndef   __ALIGNED
#define __ALIGNED(x)      __attribute__((aligned(x)))
#endif
#ifndef   __PACKED
#define __PACKED          __attribute__((packed, aligned(1)))
#endif
#ifndef   __PACKED_STRUCT
#define __PACKED_STRUCT   struct __attribute__((packed, aligned(1)))
#endif
#ifndef   __PACKED_UNION
#define __PACKED_UNION    union __attribute__((packed, aligned(1)))
#endif


// IAR Compiler
#elif defined ( __ICCARM__ )
#if (__VER__ >= 8000000)
#define __ICCARM_V8_ 1
#else
#define __ICCARM_V8_ 0
#endif

#ifndef __ALIGNED
#if __ICCARM_V8_
#define __ALIGNED(x)        __attribute__((aligned(x)))
#elif (__VER__ >= 7080000)
/* Needs IAR language extensions */
#define __ALIGNED(x)        __attribute__((aligned(x)))
#else
#warning No compiler specific solution for __ALIGNED.__ALIGNED is ignored.
#define __ALIGNED(x)
#endif
#endif
#ifndef __PACKED
#if __ICCARM_V8_
#define __PACKED            __attribute__((packed, aligned(1)))
#else
/* Needs IAR language extensions */
#define __PACKED            __packed
#endif
#endif

#ifndef   __PACKED_STRUCT
#if __ICCARM_V8_
#define __PACKED_STRUCT     struct __attribute__((packed, aligned(1)))
#else
/* Needs IAR language extensions */
#define __PACKED_STRUCT     __packed struct
#endif
#endif

#ifndef   __PACKED_UNION
#if __ICCARM_V8_
#define __PACKED_UNION      union __attribute__((packed, aligned(1)))
#else
/* Needs IAR language extensions */
#define __PACKED_UNION      __packed union
#endif
#endif


#else
#error Unknown compiler.
#endif


#endif  //__PROCESSORS_COMPILER_H

