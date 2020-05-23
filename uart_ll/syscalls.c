/*------------------------------------------------------------------------------
 -------------------------------------------------------------------------------
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 --                       THIS FILE IS UNCLASSIFIED                           --
 --  UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED   UNCLASSIFIED --
 -------------------------------------------------------------------------------
 -----------------------------------------------------------------------------*/

/**
 * @file    syscalls.c
 * @brief   This file contains low level system calls and stubs.
 *
 * This file contains low level system calls and stubs.  See:
 * http://embdev.net/topic/177530
 * http://www.utasker.com/forum/index.php?topic=748.0
 * Also see documentation in share/doc/arm-arm-none-eabi/html/libc/Syscalls.html
 *
 * @date    May 12, 2020
 * @author  Harry Rostovtsev
 * @email   Harry.Rostovtsev@NGC.com
 * Copyright 2020, Northrop Grumman Innovation Systems, Inc.
 * All other rights reserved.
 *
 * NORTHROP GRUMMAN PROPRIETARY LEVEL I
 * This information contains proprietary data and should not be released or
 * distributed without the express written approval of Northrop Grumman
 * Innovation Systems, Inc. This document contains private or privileged
 * information and/or trade secrets, which is/are disclosed in confidence.
 * This information may be used, duplicated, or disclosed only to the extent as
 * specifically authorized in writing by Northrop Grumman Innovation Systems,
 * Inc.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

#include "cmsis_gcc.h"

#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_ll_usart.h"
//#include "uart.h"
//#include "board_defs.h"




/* Compile-time called macros ------------------------------------------------*/
/* Private typedefs ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#ifndef STDOUT_USART
#define STDOUT_USART USART3
#endif

#ifndef STDERR_USART
#define STDERR_USART USART3
#endif

#ifndef STDIN_USART
#define STDIN_USART USART3
#endif

#undef errno

/* Private macros ------------------------------------------------------------*/
/* Private variables and Local objects ---------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//static char *heap_end;
extern int errno;
extern int  _end;

/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
 * @brief   Increase program data space.
 * As malloc and related functions depend on this, it is useful to have a working
 * implementation. The following suffices for a standalone system; it exploits
 * the symbol _end automatically defined by the GNU linker.
 *
 * @return  NULL
 */
caddr_t _sbrk(int incr)
{
    static unsigned char *heap = NULL;
    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }

    heap += incr;

    /* Returning NULL causes a hard fault any time this function is called
     * That's a good thing since we don't want to use any dynamic memory allocation */
    return NULL;
    /*return ((caddr_t) prev_heap);*/
}

/**
 * @brief   Exit a program without cleaning up files.
 *
 * If your system doesn’t provide this, it is best to avoid linking with
 * subroutines that require it (exit, system).
 *
 * @return  None
 */
//void _exit(int status)
//{
//    printf("_exit called with parameter %d\n", status);
//    while(1) {;} // Prevent the processor from faulting
//}

/**
 * @brief   Get Process-ID
 *
 * This is sometimes used to generate strings unlikely to conflict with other
 * processes. Minimal implementation, for a system without processes:
 *
 * @return  1
 */
//int _getpid(void)
//{
//    return(1);
//}

/**
 * @brief   Send a signal.
 *
 * Minimal implementation for a system without processes.
 *
 * @return  -1
 */
//int _kill(int pid, int sig)
//{
//    (void)pid;
//    (void)sig; /* avoid warnings */
//    errno = EINVAL;
//    return(-1);
//}

/**
 * @brief   Close a file
 *
 * Minimal implementation for a system without files.
 *
 * @return  -1
 */
int _close(int file)
{
    (void)file;
    return(-1);
}

/**
 * @brief   Status of an open file.
 *
 * For consistency with other minimal implementations in these examples, all
 * files are regarded as character special devices. The sys/stat.h header file
 * required is distributed in the include subdirectory for this C library.
 *
 * Minimal implementation for a system without files.
 *
 * @return  0
 */
int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return(0);
}

/**
 * @brief   Query whether output stream is a terminal.
 *
 * For consistency with the other minimal implementations, which only support
 * output to stdout.
 *
 * Minimal implementation for a system without files.
 *
 * @return  1
 */
int _isatty(int file)
{
    (void)file;
    return(1);
}

/**
 * @brief   Set position in a file.
 *
 * Minimal implementation for a system without files.
 *
 * @return  0
 */
int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return(0);
}

/**
 * @brief   Read from a file
 *
 * This implementation remaps reading from a file to read from a debug UART.
 * This allows functions like scanf to work.
 *
 * @warning: this is a blocking function which depends on the speed of the
 * UART
 *
 * @return  number of characters read
 */
int _read(int file, char *ptr, int len)
{
    int n;
    int num = 0;
    switch (file) {
        case STDIN_FILENO:
            for( n = 0; n < len; n++ ) {
                /* Wait until character has been received */
//                while( !( STDIN_USART->STAT & LPUART_STAT_RDRF_MASK ) ) {}
//                *ptr++ = STDIN_USART->DATA;
                num++;
            }
            break;
        default:
            errno = EBADF;
            return(-1);
    }
    return(num);
}

/**
 * @brief   Write to a file
 *
 * This implementation remaps writing to a file to write to a debug UART.
 * This allows functions like printf to work.
 *
 * @warning: this is a blocking function which depends on the speed of the
 * UART
 *
 * @return  number of characters written
 */
int _write(int file, char *ptr, int len) {
    switch (file) {
        case STDOUT_FILENO: /* stdout */
        case STDERR_FILENO: /* stderr */
                while( len-- ) {
                    while( !( LL_USART_IsActiveFlag_TC(STDERR_USART)) ) {}
                    LL_USART_TransmitData8(STDERR_USART, *ptr++);
                }
            break;
        default:
            errno = EBADF;
            return(-1);
    }
    return(len);
}

/**
 * @brief   Get end of the heap memory
 *
 * Minimal implementation for malloc to exist
 *
 * @return  pointer to end of heap
 */
//char* get_heap_end(void)
//{
//    return ((char*) heap_end);
//}
//
//
///**
// * @brief   Get end of the stack memory
// *
// * Minimal implementation for malloc to exist
// *
// * @return  pointer to end of heap
// */
//char* get_stack_top(void)
//{
//    return ((char*) __get_MSP());
//}

