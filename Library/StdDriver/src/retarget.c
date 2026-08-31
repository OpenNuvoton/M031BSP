/**************************************************************************//**
 * @file     retarget.c
 * @version  V0.10
 * @brief    Debug Port and Semihost Setting Source File
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*
 * Compiler identification used in this file
 * -----------------------------------------
 * ARMCC5                 : __CC_ARM, __ARMCC_VERSION
 * ARMCLANG6 (Keil/VSCode): __ARMCC_VERSION
 * GCC (VSCode/Eclipse)   : __GNUC__ and no __ARMCC_VERSION
 * IAR                    : __ICCARM__
 *
 * Note:
 * Keil / VSCode / Eclipse are IDE or build environments.
 * Conditional compilation in retarget.c is selected by the actual compiler.
 */

/* GCC only: newlib/newlib-nano support */
#if defined(__GNUC__) && !defined(__ARMCC_VERSION) && !defined(OS_USE_SEMIHOSTING)
#include <sys/stat.h>
#endif

#if defined (__ICCARM__)
    #if (__VER__ >= 9000000)
        #include <LowLevelIOInterface.h>
    #endif
    #pragma diag_suppress=Pm150
#endif

/* ARM Compiler 5 */
#if defined(__CC_ARM)
    #if (__ARMCC_VERSION < 400000)
    #else
        /* Insist on keeping widthprec, to avoid X propagation by benign code in C-lib */
        #pragma import _printf_widthprec
    #endif
#endif

#ifndef DEBUG_PORT
    #define DEBUG_PORT   UART1
#endif

#define BUF_SIZE     512


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if !(defined(__ICCARM__) && (__VER__ >= 6010000))
#if (__ARMCC_VERSION < 6040000)
struct __FILE
{
    /* Add whatever you need here */
};
#else
#if !defined(__MICROLIB)
    #if (__OPTIMIZE__ == -O0)
        __asm(".global __ARM_use_no_argv\n\t" "__ARM_use_no_argv:\n\t");
    #endif /* (__OPTIMIZE__ == -O0) */
#endif /* !defined(__MICROLIB) */
#endif /* (__ARMCC_VERSION < 6040000) */

#elif(__VER__ >= 8000000)
struct __FILE
{
    int handle; /* Add whatever you need here */
};
#endif /* !(defined(__ICCARM__) && (__VER__ >= 6010000)) */


/* [MISRA8.4] Provide compatible declarations for external linkage objects. */
extern FILE __stdout;
extern FILE __stdin;

FILE __stdout;
FILE __stdin;

#if defined(DEBUG_ENABLE_SEMIHOST)
static volatile int32_t g_ICE_Connected = 1;
#endif
int kbhit(void);
int IsDebugFifoEmpty(void);
void _ttywrch(int ch);
/* cppcheck-suppress misra-c2012-21.2 */
int fputc(int ch, FILE *stream);

/* GCC only */
#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
    #if !defined(OS_USE_SEMIHOSTING)
        int _read(int fd, char *ptr, int len);
    #endif

    int _write(int fd, char *ptr, int len);
#endif

/* ARMCC / ARMCLANG / IAR */
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
    /* cppcheck-suppress misra-c2012-21.2 */
    int fgetc(FILE *stream);
    /* cppcheck-suppress misra-c2012-21.2 */
    int ferror(FILE *stream);
#endif

char GetChar(void);
void SendChar_ToUART(int ch);
void SendChar(int ch);
enum { r0, r1, r2, r3, r12, lr, pc, psr};


/**
 * @brief       Helper function to dump register while hard fault occurred
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @details     This function is implement to print r0, r1, r2, r3, r12, lr, pc, psr
 */
static void DumpStack(const uint32_t stack[])
{
    /* [MISRA2.7] Explicitly mark unused parameter (debug prints are disabled). */
    (void)stack; 

    /*
        printf("r0 =0x%x\n", stack[r0]);
        printf("r1 =0x%x\n", stack[r1]);
        printf("r2 =0x%x\n", stack[r2]);
        printf("r3 =0x%x\n", stack[r3]);
        printf("r12=0x%x\n", stack[r12]);
        printf("lr =0x%x\n", stack[lr]);
        printf("pc =0x%x\n", stack[pc]);
        printf("psr=0x%x\n", stack[psr]);
    */
}

#if defined(DEBUG_ENABLE_SEMIHOST)

/* The static buffer is used to speed up the semihost */
static char g_buf[16];
static char g_buf_len = 0;

/**
 *
 * @brief      The function to process semihosted command
 * @param[in]  n32In_R0  : semihost register 0
 * @param[in]  n32In_R1  : semihost register 1
 * @param[out] pn32Out_R0: semihost register 0
 * @retval     0: No ICE debug
 * @retval     1: ICE debug
 *
 */
int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    /* [MISRA2.7] Mark unused if not used by implementation. */
    (void)n32In_R1;

    if (g_ICE_Connected)
    {
        /* [MISRA15.6] Always use compound statements. */
        if (pn32Out_R0 != NULL)
        {
            *pn32Out_R0 = n32In_R0;
        }
        return 1;
    }

    return 0;
}

#else // defined(DEBUG_ENABLE_SEMIHOST)

#if !(defined ( __GNUC__ ) && !defined (__ARMCC_VERSION))

int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);
/* [MISRA2.7] Parameters are intentionally unused in semihost return stub. */
int32_t SH_Return(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
{
    (void)n32In_R0;
    (void)n32In_R1;
    (void)pn32Out_R0;

    return 0L;
}

#endif

#endif /* defined(DEBUG_ENABLE_SEMIHOST) */
/* ARMCC / ARMCLANG / IAR */
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
    extern int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);

    #if defined(__ICCARM__)
        __WEAK
    #else
        __attribute__((weak))
    #endif

    /**
     * @static_deviation
     * <b>Rule:</b>          MISRA C:2012 Rule 8.2<br>
     * <b>Justification:</b> Parameters lr, msp, psp are already named in this prototype;
     *                       the preceding conditional __WEAK / __attribute__((weak))
     *                       qualifier confuses the MISRA addon's simplified token stream
     *                       into misreading the parameter list as unnamed. This is a
     *                       tool-parsing artifact of the weak-attribute macro, not a
     *                       missing parameter name.
     */
    /* cppcheck-suppress misra-c2012-8.2 */
    uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp);
#else
    extern int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0);
#endif

#if defined(__ICCARM__)
    __WEAK
#else
    __attribute__((weak))
#endif
uint32_t ProcessHardFault(uint32_t lr, uint32_t msp, uint32_t psp)
{
    uint32_t *sp;
    uint32_t inst;

    /* It is caused by hardfault. Just process the hard fault */
    /* TODO: Implement your hardfault handle code here */

    /* Check the used stack */
#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)

    if ((lr & 0x40UL) != 0UL)
    {
#endif

        /* Secure stack used */
        if ((lr & 4UL) != 0UL)
        {
            sp = (uint32_t *)psp;
        }
        else
        {
            sp = (uint32_t *)msp;
        }

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
    }
    else
    {
        /* Non-secure stack used */
        /* [MISRA15.6] Always use compound statements. */
        if ((lr & 4UL) != 0UL)
        {
            sp = (uint32_t *)__TZ_get_PSP_NS();
        }
        else
        {
            sp = (uint32_t *)__TZ_get_MSP_NS();
        }
    }
#endif

    /* Get the instruction caused the hardfault */
    inst = M16(sp[6]);

    /* [MISRA10.4] Use unsigned constants to match essential type category. */
    if (inst == 0xBEABUL)
    {
        /*
            If the instruction is 0xBEAB, it means it is caused by BKPT without ICE connected.
            We still return for output/input message to UART.
        */
#if defined(DEBUG_ENABLE_SEMIHOST)
        g_ICE_Connected = 0;         /* Set a flag for ICE offline */
#endif			
			
        sp[6] += 2UL;         /* Return to next instruction */
        return lr;            /* Keep lr in R0 */
    }

    /* Do not use standard I/O in fault context. */
    /* printf("  HardFault!\n\n"); */ /* Removed */

    DumpStack(sp);

    /* Explicitly reference sp to avoid unused warning in some toolchains/configs. */
    (void)*sp;

    /* Explicit infinite loop. No return statement is needed because this path never exits. */
    for (;;)
    {
        /* stay here */
    }
}



/**
 * @brief    Routine to send a char
 *
 * @param[in] ch  A character data writes to debug port
 *
 * @details  Send a target char to UART debug port .
 */
#ifndef NONBLOCK_PRINTF
void SendChar_ToUART(int ch)
{
    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
    {
        /* wait */ /* [MISRA15.6] */
    }

    if ((char)ch == '\n')
    {
        DEBUG_PORT->DAT = '\r';

        while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
        {
            /* wait */ /* [MISRA15.6] */
        }
    }

    DEBUG_PORT->DAT = (uint32_t)ch;
}

#else

/* Non-block implement of send char */
void SendChar_ToUART(int ch)
{
    static uint8_t u8Buf[BUF_SIZE] = {0};
    static int32_t i32Head = 0;
    static int32_t i32Tail = 0;
    int32_t i32Tmp;

    /* Only flush the data in buffer to UART when ch == 0 */
    if (ch)
    {
        // Push char

        if (ch == '\n')
        {
            i32Tmp = i32Head + 1;

            /* [MISRA15.6] Always use compound statements. */
            if (i32Tmp > BUF_SIZE) 
            {
                i32Tmp = 0;
            }

            if (i32Tmp != i32Tail)
            {
                u8Buf[i32Head] = '\r';
                i32Head = i32Tmp;
            }
        }

        i32Tmp = i32Head + 1;

        /* [MISRA15.6] Always use compound statements. */
        if (i32Tmp > BUF_SIZE) 
        {
            i32Tmp = 0;
        }

        if (i32Tmp != i32Tail)
        {
            u8Buf[i32Head] = (uint8_t)ch;
            i32Head = i32Tmp;
        }
    }
    else
    {
        /* [MISRA15.6] Always use compound statements. */
        if (i32Tail == i32Head)
        {
            return;
        }
    }

    // Pop char
    do
    {
        i32Tmp = i32Tail + 1;

        /* [MISRA15.6] Always use compound statements. */
        if (i32Tmp > (int32_t)BUF_SIZE)
        {
            i32Tmp = 0;
        }


        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) == 0UL)
        {
            DEBUG_PORT->DAT = (uint32_t)u8Buf[i32Tail];
            i32Tail = i32Tmp;
        }
        else
        {
            break; /* FIFO full */ /* [MISRA15.6] */
        }
    } while (i32Tail != i32Head);
}
#endif /* else for NONBLOCK_PRINTF */


/**
 * @brief    Routine to send a char
 *
 * @param[in] ch : A character data writes to debug port
 *
 * @returns  Send value from UART debug port or semihost
 *
 * @details  Send a target char to UART debug port or semihost.
 */

#if !defined( __ICCARM__ )
    #define __WEAK    __attribute__((weak))
#endif
__WEAK void SendChar(int ch)
{
#if defined(DEBUG_ENABLE_SEMIHOST)
    g_buf[g_buf_len++] = ch;
    g_buf[g_buf_len] = '\0';

    if (g_buf_len + 1 >= sizeof(g_buf) || ch == '\n' || ch == '\0')
    {
        /* Send the char */
        if (g_ICE_Connected)
        {
            if (SH_DoCommand(0x04, (int)g_buf, NULL) != 0)
            {
                g_buf_len = 0;

                return;
            }
        }
        else
        {
#if (DEBUG_ENABLE_SEMIHOST == 1) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=1
        int i;


        for (i = 0; i < g_buf_len; i++)
        {
            SendChar_ToUART(g_buf[i]);
        }

#endif
            g_buf_len = 0;
        }
    }

#else
    SendChar_ToUART(ch);
#endif /* DEBUG_ENABLE_SEMIHOST */
}


/**
 * @brief    Routine to get a char
 *
 *
 * @returns  Get value from UART debug port or semihost
 *
 * @details  Wait UART debug port or semihost to input a char.
 */
char GetChar(void)
{
#ifdef DEBUG_ENABLE_SEMIHOST

    if (g_ICE_Connected)
    {
#if defined (__ICCARM__)
        int nRet;

        while (SH_DoCommand(0x7, 0, (int32_t *)&nRet) != 0)
        {
            /* [MISRA15.6] Always use compound statements. */
            if (nRet != 0)
            {
                return (char)nRet;
            }
        }
#else
        int nRet;

        while (SH_DoCommand(0x101, 0, (int32_t *)&nRet) != 0)
        {
            if (nRet != 0)
            {
                SH_DoCommand(0x07, 0, (int32_t *)&nRet);
                return (char)nRet;
            }
        }


#endif

    }
    else
    {

#if (DEBUG_ENABLE_SEMIHOST == 1) // Re-direct to UART Debug Port only when DEBUG_ENABLE_SEMIHOST=1

        /* Use debug port when ICE is not connected at semihost mode */
        while (!g_ICE_Connected)
        {
            if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
            {
                return ((char)DEBUG_PORT->DAT);
            }
        }

#endif
    }

    return (0);

#else

    while (1)
    {
        if ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U)
        {
            return ((char)DEBUG_PORT->DAT);
        }
    }

#endif
}


/**
 * @brief    Check any char input from UART
 *
 *
 * @retval   1: No any char input
 * @retval   0: Have some char input
 *
 * @details  Check UART RSR RX EMPTY or not to determine if any char input from UART
 */
int kbhit(void)
{
    return !((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0U);
}


/**
 * @brief  Check if debug message finished
 *
 * @return   1 Message is finished.
 *           0 Message is transmitting.
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */
int IsDebugFifoEmpty(void)
{
    return ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) != 0U);
}


/**
 * @brief    C library retargetting
 *
 * @param[in]  ch  Write a character data
 *
 * @details  Check if message finished (FIFO empty of debug port)
 */
void _ttywrch(int ch)
{
#if defined(__ICCARM__) && (__VER__ >= 8000000)
    (void)__stdout.handle;
    (void)__stdin.handle;
#endif

    SendChar(ch);
    return;
}


/**
 * @brief      Write character to stream
 *
 * @param[in]  ch       Character to be written. The character is passed as its int promotion.
 * @param[in]  stream   Pointer to a FILE object that identifies the stream where the character is to be written.
 *
 * @returns    If there are no errors, the same character that has been written is returned.
 *             If an error occurs, EOF is returned and the error indicator is set (see ferror).
 *
 * @details    Writes a character to the stream and advances the position indicator.\n
 *             The character is written at the current position of the stream as indicated \n
 *             by the internal position indicator, which is then advanced one character.
 *
 * @note       The above descriptions are copied from http://www.cplusplus.com/reference/clibrary/cstdio/fputc/.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.2<br>
 * <b>Justification:</b> fputc is a standard C library function name that must be
 *                       redeclared/redefined here by design: this is the retarget
 *                       layer that hooks the C runtime's low-level character output
 *                       to the UART debug port / semihosting channel. Renaming it
 *                       would break the C library's retargeting mechanism, which
 *                       depends on this exact reserved name being provided.
 */
#if defined (__ICCARM__) && (__VER__ >= 9000000)
size_t __write(int handle, const unsigned char *buffer, size_t size)
{
    size_t nChars = 0;

    if (buffer == 0)
    {
        /*
        * This means that we should flush internal buffers.  Since we
        * don't we just return.  (Remember, "handle" == -1 means that all
        * handles should be flushed.)
        */
        return 0;
    }

    /* This template only writes to "standard out" and "standard err",
    * for all other file handles it returns failure. */
    if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size != 0; --size)
    {
        SendChar(*buffer++);
        ++nChars;
    }

    return nChars;
}
#else
int fputc(int ch, FILE *stream)
{
    (void)stream; /* [MISRA2.7] */
    SendChar(ch);
    return ch;
}
#endif

/* GCC only */
#if defined(__GNUC__) && !defined(__ARMCC_VERSION)

#if defined (OS_USE_SEMIHOSTING)

#else

int _write(int fd, char *ptr, int len)
{
    int i = len;

    (void)fd; /* [MISRA2.7] */

    while (i--)
    {
        while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
        {
            /* wait */ /* [MISRA15.6] */
        }

        if (*ptr == '\n')
        {
            DEBUG_PORT->DAT = '\r';

            while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) != 0UL)
            {
                /* wait */ /* [MISRA15.6] */
            }
        }

        DEBUG_PORT->DAT = *ptr++;
    }

    return len;
}


int _read(int fd, char *ptr, int len)
{
    (void)fd;  /* [MISRA2.7] */
    (void)len; /* [MISRA2.7] */

    while ((DEBUG_PORT->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) != 0UL)
    {
        /* wait */ /* [MISRA15.6] */
    }


    *ptr = DEBUG_PORT->DAT;
    return 1;
}

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
int _close(int file) 
{
    (void)file; /* [MISRA2.7] */
    return -1;
}

int _lseek(int file, int ptr, int dir) 
{
    (void)file; /* [MISRA2.7] */
    (void)ptr;  /* [MISRA2.7] */
    (void)dir;  /* [MISRA2.7] */

    return 0;
}

int _fstat(int file, struct stat *st) 
{
    (void)file; /* [MISRA2.7] */

    if (st != NULL)
    {
        st->st_mode = S_IFCHR;
    }

    return 0;
}

int _isatty(int file) 
{
    (void)file; /* [MISRA2.7] */
    return 1;
}

int _kill(int pid, int sig) 
{
    (void)pid; /* [MISRA2.7] */
    (void)sig; /* [MISRA2.7] */
    return -1;
}

int _getpid(void) 
{
    return 1;
}
#endif

#else

/**
 * @brief      Get character from UART debug port or semihosting input
 *
 * @param[in]  stream   Pointer to a FILE object that identifies the stream on which the operation is to be performed.
 *
 * @returns    The character read from UART debug port or semihosting
 *
 * @details    For get message from debug port or semihosting.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.2<br>
 * <b>Justification:</b> fgetc is a standard C library function name that must be
 *                       redeclared/redefined here by design: this is the retarget
 *                       layer that hooks the C runtime's low-level character input
 *                       to the UART debug port / semihosting channel. Renaming it
 *                       would break the C library's retargeting mechanism, which
 *                       depends on this exact reserved name being provided.
 */
#if defined (__ICCARM__) && (__VER__ >= 9000000)
size_t __read(int handle, unsigned char *buffer, size_t size)
{
    /* Remove the #if #endif pair to enable the implementation */
    int nChars = 0;

    /* This template only reads from "standard in", for all other file
     * handles it returns failure. */
    if (handle != _LLIO_STDIN)
    {
        return _LLIO_ERROR;
    }

    for (/* Empty */; size > 0; --size)
    {
        int c = GetChar();

        if (c < 0)
        {
            break;
        }
#if (STDIN_ECHO != 0)
        SendChar(c);
#endif

        *buffer++ = c;
        ++nChars;
    }

    return nChars;
}

long __lseek(int handle, long offset, int whence)
{
    return -1;
}
#else
int fgetc(FILE *stream);
int fgetc(FILE *stream)
{
    (void)stream; /* [MISRA2.7] */
    return ((int)GetChar());
}
#endif

/**
 * @brief      Check error indicator
 *
 * @param[in]  stream   Pointer to a FILE object that identifies the stream.
 *
 * @returns    If the error indicator associated with the stream was set, the function returns a nonzero value.
 *             Otherwise, it returns a zero value.
 *
 * @details    Checks if the error indicator associated with stream is set, returning a value different
 *             from zero if it is. This indicator is generally set by a previous operation on the stream that failed.
 *
 * @note       The above descriptions are copied from http://www.cplusplus.com/reference/clibrary/cstdio/ferror/.
 *
 * @static_deviation
 * <b>Rule:</b>          MISRA C:2012 Rule 21.2<br>
 * <b>Justification:</b> ferror is a standard C library function name that must be
 *                       redeclared/redefined here by design: this is the retarget
 *                       layer that hooks the C runtime's stream error-indicator query
 *                       to this BSP's debug/semihosting channel. Renaming it would
 *                       break the C library's retargeting mechanism, which depends
 *                       on this exact reserved name being provided.
 */
int ferror(FILE *stream)
{
    (void)stream; /* [MISRA2.7] */
    return EOF;
}
#endif


#ifdef DEBUG_ENABLE_SEMIHOST
#ifdef __ICCARM__
void __exit(int return_code)
{

    /* Check if link with ICE */
    if (SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0)
        {
            /* wait */ /* [MISRA15.6] */
        }

    }

label:
    goto label;  /* Endless loop */
}

#else

void _sys_exit(int return_code)
{
    (void)return_code; /* [MISRA2.7] */

    /* Check if link with ICE */
    if (SH_DoCommand(0x18, 0x20026, NULL) == 0)
    {
        /* Make sure all message is print out */
        while (IsDebugFifoEmpty() == 0)
        {
            /* wait */ /* [MISRA15.6] */
        }

    }

label:
    goto label;  /* Endless loop */
}

#endif
#endif
