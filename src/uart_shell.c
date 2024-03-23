/**
 ********************************************************************************
 * @file    uart_shell.h
 * @author  micahjp5158
 * @date    3-17-2024
 * @brief   Handles all primary UART interfacing, including the system call overwrites
 *          to reroute standard IO to the shell console.
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "uart_shell.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
// UART TX pin configuration
#define UART_SHELL_TX_GPIO_PORT   GPIOA
#define UART_SHELL_TX_GPIO_PIN    GPIO_PIN_2
#define UART_SHELL_TX_GPIO_MODE   GPIO_MODE_AF_PP;
#define UART_SHELL_TX_GPIO_SPEED  GPIO_SPEED_FAST;
#define UART_SHELL_TX_GPIO_PULL   GPIO_PULLUP;
#define UART_SHELL_TX_GPIO_AF     GPIO_AF7_USART2

// UART RX pin configuration
#define UART_SHELL_RX_GPIO_PORT   GPIOA
#define UART_SHELL_RX_GPIO_PIN    GPIO_PIN_3
#define UART_SHELL_RX_GPIO_MODE   GPIO_MODE_AF_PP;
#define UART_SHELL_RX_GPIO_SPEED  GPIO_SPEED_FAST;
#define UART_SHELL_RX_GPIO_PULL   GPIO_PULLUP;
#define UART_SHELL_RX_GPIO_AF     GPIO_AF7_USART2

// UART configuration
#define UART_SHELL_CFG_USART_ID       USART2
#define UART_SHELL_CFG_BAUDRATE       115200
#define UART_SHELL_CFG_WORD_LEN       UART_WORDLENGTH_8B
#define UART_SHELL_CFG_STOP_BITS      UART_STOPBITS_1
#define UART_SHELL_CFG_PARITY         UART_PARITY_NONE
#define UART_SHELL_CFG_FLOW_CTRL      UART_HWCONTROL_NONE
#define UART_SHELL_CFG_MODE           UART_MODE_TX_RX
#define UART_SHELL_CFG_OVERSAMPLING   UART_OVERSAMPLING_16

// STDIO file descriptors
#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/
UART_HandleTypeDef UART_Shell_Handle;
TaskHandle_t UART_Shell_Task_Handler;

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void uart_shell_task_handler(void *parameters);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void uart_shell_task_handler(void *parameters)
{
  printf("Shell task started\n");
  while(1){
    // TODO
  }
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void uart_shell_init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  // Initialize GPIOA clock
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure TX pin
  gpio_init.Pin = UART_SHELL_TX_GPIO_PIN;
  gpio_init.Mode = UART_SHELL_TX_GPIO_MODE;
  gpio_init.Speed = UART_SHELL_TX_GPIO_SPEED;
  gpio_init.Pull = UART_SHELL_TX_GPIO_PULL;
  gpio_init.Alternate = UART_SHELL_TX_GPIO_AF;
  HAL_GPIO_Init(UART_SHELL_TX_GPIO_PORT, &gpio_init);

  // Configure RX pin
  gpio_init.Pin = UART_SHELL_RX_GPIO_PIN;
  gpio_init.Mode = UART_SHELL_RX_GPIO_MODE;
  gpio_init.Speed = UART_SHELL_RX_GPIO_SPEED;
  gpio_init.Pull = UART_SHELL_RX_GPIO_PULL;
  gpio_init.Alternate = UART_SHELL_RX_GPIO_AF;
  HAL_GPIO_Init(UART_SHELL_RX_GPIO_PORT, &gpio_init);

  // Initialize UART2 clock
  __HAL_RCC_USART2_CLK_ENABLE();

  // Initialize UART peripheral
  UART_Shell_Handle.Instance        = UART_SHELL_CFG_USART_ID;
  UART_Shell_Handle.Init.BaudRate   = UART_SHELL_CFG_BAUDRATE;
  UART_Shell_Handle.Init.StopBits   = UART_SHELL_CFG_STOP_BITS;
  UART_Shell_Handle.Init.Parity     = UART_SHELL_CFG_PARITY;
  UART_Shell_Handle.Init.HwFlowCtl  = UART_SHELL_CFG_FLOW_CTRL;
  UART_Shell_Handle.Init.Mode       = UART_SHELL_CFG_MODE;
  UART_Shell_Handle.Init.OverSampling = UART_SHELL_CFG_OVERSAMPLING;

  if (HAL_UART_Init(&UART_Shell_Handle) != HAL_OK)
  {
    // TODO error handling
    return;
  }

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);

  // Create the shell task
  BaseType_t status;
  status = xTaskCreate(uart_shell_task_handler,
                      "UART Shell task",
                      200,
                      NULL,
                      2,
                      &UART_Shell_Task_Handler);

  configASSERT(status == pdPASS);
}

/* Modified system calls to support using printf over UART */
/* Based on https://shawnhymel.com/1873/how-to-use-printf-on-stm32/ */
int _isatty(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 1;

  errno = EBADF;
  return 0;
}

int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    hstatus = HAL_UART_Transmit(&UART_Shell_Handle, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _close(int fd) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
    return 0;

  errno = EBADF;
  return -1;
}

int _lseek(int fd, int ptr, int dir) {
  (void) fd;
  (void) ptr;
  (void) dir;

  errno = EBADF;
  return -1;
}

int _read(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  if (fd == STDIN_FILENO) {
    hstatus = HAL_UART_Receive(&UART_Shell_Handle, (uint8_t *) ptr, 1, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return 1;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}
