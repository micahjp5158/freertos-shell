/**
 ********************************************************************************
 * @file    uart_shell.h
 * @author  micahjp5158
 * @date    3-17-2024
 * @brief   Handles all primary UART interfacing, including the system call overwrites
 *          to reroute standard IO to the shell console.
 ********************************************************************************
 */

#ifndef __UART_SHELL_H__
#define __UART_SHELL_H__

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include <sys/stat.h>

/************************************
 * MACROS AND DEFINES
 ************************************/

/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/

void uart_shell_init(void);

/* Modified system calls to support using printf over UART */
/* Based on https://shawnhymel.com/1873/how-to-use-printf-on-stm32/ */
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#ifdef __cplusplus
}
#endif

#endif // __UART_SHELL_H__