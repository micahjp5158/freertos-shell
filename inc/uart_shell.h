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
typedef enum UART_SHELL_REG_CMD_STATUS {
  UART_SHELL_REG_CMD_OK,
  UART_SHELL_REG_CMD_ERROR_MAX_CMDS,
  UART_SHELL_REG_CMD_ERROR_NULL_PTR,
  UART_SHELL_REG_CMD_ERROR_BUFFER_OVERFLOW,
  UART_SHELL_REG_CMD_ERROR_DUPLICATE_CMD,
  NUM_UART_SHELL_REG_CMD_STATUS
} UART_SHELL_REG_CMD_STATUS_T;

typedef void (*UART_Shell_Cmd_Callback)(char *);

/************************************
 * EXPORTED VARIABLES
 ************************************/

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void uart_shell_init(void);

/**
 * @brief Register a command in the UART shell. If a command entered by the user
 *        matches the provided identifier, the provided callback is called to
 *        process the command further.
 * @param id: Short identifer string. Commands entered by the user will begin with
 *            this string, which will be compared to the registered commands' ids
 *            to call the appropriate callback.
 * @param help_str: A string generally describing what the command does. Will be
 *                  printed by the UART shell when receiving the help command.
 * @param callback: Callback function pointer that will be called when the user
 *                  enters a command matching this command's id.
 * @return: UART_SHELL_REG_CMD_OK if successfully registered, error code otherwise.
 ********************************************************************************/
UART_SHELL_REG_CMD_STATUS_T uart_shell_register_cmd(
                              char *id,
                              char *help_str,
                              UART_Shell_Cmd_Callback callback);

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