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

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
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

// UART configuration
#define UART_SHELL_CFG_USART_ID       USART2
#define UART_SHELL_CFG_BAUDRATE       9600
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

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/

/************************************
 * STATIC FUNCTIONS
 ************************************/

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
}