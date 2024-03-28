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

#include "ringbuf.h"

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_ll_usart.h"

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
#include <string.h>

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

// UART RX buffer
#define UART_SHELL_RX_BUF_SIZE  64

// UART interrupt configuration
#define UART_SHELL_IRQ_N        USART2_IRQn
#define UART_SHELL_IRQ_Handler  USART2_IRQHandler

// UART configuration
#define UART_SHELL_CFG_USART_ID       USART2
#define UART_SHELL_CFG_BAUDRATE       115200
#define UART_SHELL_CFG_WORD_LEN       UART_WORDLENGTH_8B
#define UART_SHELL_CFG_STOP_BITS      UART_STOPBITS_1
#define UART_SHELL_CFG_PARITY         UART_PARITY_NONE
#define UART_SHELL_CFG_FLOW_CTRL      UART_HWCONTROL_NONE
#define UART_SHELL_CFG_MODE           UART_MODE_TX_RX
#define UART_SHELL_CFG_OVERSAMPLING   UART_OVERSAMPLING_16

// FreeRTOS signals
#define UART_SHELL_SIGNAL_RX_COMPLETE         0x01
#define UART_SHELL_SIGNAL_RX_BUFFER_OVERFLOW  0x02

// STDIO file descriptors
#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

// Command registry defines
#define UART_SHELL_CMD_ID_SIZE        8
#define UART_SHELL_CMD_HELP_STR_SIZE  32
#define UART_SHELL_CMD_MAX_NUM_CMDS   8

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
// Holds a shell command's info
typedef struct{
  char cmd_id[UART_SHELL_CMD_ID_SIZE];
  char cmd_help_str[UART_SHELL_CMD_HELP_STR_SIZE];
  UART_Shell_Cmd_Callback cmd_callback;
} UART_Shell_Command_t;

/************************************
 * STATIC VARIABLES
 ************************************/
// Command registry
static UART_Shell_Command_t uart_shell_cmds[UART_SHELL_CMD_MAX_NUM_CMDS];
static uint8_t uart_shell_num_cmds = 0;

// UART RX ring buffer
static RingBuf_Handle_t Rx_RingBuf_Handle;
static uint8_t rx_buf[UART_SHELL_RX_BUF_SIZE];

// UART Shell command input_buffer
static char uart_shell_cmd_input_buf[UART_SHELL_RX_BUF_SIZE];

// UART peripheral handler
static UART_HandleTypeDef UART_Shell_Handle;

// FreeRTOS task handles
static TaskHandle_t UART_Shell_Task_Handle;
static uint32_t     UART_Shell_Task_Notifications;

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void uart_shell_process_cmd();
static void uart_shell_task_handler(void *parameters);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void uart_shell_process_cmd()
{
  char *cmd_id = strtok(uart_shell_cmd_input_buf, " ");

  // Check for help command
  if (strcmp(cmd_id, "help") == 0)
  {
    printf("TODO: Help screen\n");
    return;
  }

  // Check registered commands
  for (int i = 0; i < uart_shell_num_cmds; i++)
  {
    if (strcmp(cmd_id, uart_shell_cmds[i].cmd_id) == 0)
    {
      uart_shell_cmds[i].cmd_callback(uart_shell_cmd_input_buf);
      return;
    }
  }

  // Unknown command
  printf("Unrecognized command \'%s\'. Enter \'help\' for a list of commands.\n", cmd_id);
}

static void uart_shell_task_handler(void *parameters)
{
  printf("Shell task started\n");

  while(1){
    // Shell input cursor to prompt for input
    printf("> ");

    // Wait for shell event
    xTaskNotifyWait(UINT32_MAX, 0, &UART_Shell_Task_Notifications, portMAX_DELAY);

    // Disable RX interrupts while handling command / errors, since
    // accessing RX ring buffer includes critical sections
    LL_USART_DisableIT_RXNE(UART_SHELL_CFG_USART_ID);

    // Process received command
    if ((UART_Shell_Task_Notifications & UART_SHELL_SIGNAL_RX_COMPLETE) == UART_SHELL_SIGNAL_RX_COMPLETE)
    {
      // Copy the command from the ringbuffer
      char data;
      uint8_t i = 0;
      while (ringbuf_get(&Rx_RingBuf_Handle, &data) == RINGBUF_STATUS_OK)
      {
        uart_shell_cmd_input_buf[i] = data;
        i++;
      }

      // Process command
      uart_shell_process_cmd();

      // Clear the command buffer for next use
      memset(uart_shell_cmd_input_buf, 0, UART_SHELL_RX_BUF_SIZE * sizeof(char));
    }

    // Process RX buffer overflow
    if ((UART_Shell_Task_Notifications & UART_SHELL_SIGNAL_RX_BUFFER_OVERFLOW) == UART_SHELL_SIGNAL_RX_BUFFER_OVERFLOW)
    {
      printf("\nERROR: RX buffer overflow. Limit command input to %d characters.\n", UART_SHELL_RX_BUF_SIZE);
      ringbuf_clear(&Rx_RingBuf_Handle);
    }

    // Reenable RX interrupts
    LL_USART_EnableIT_RXNE(UART_SHELL_CFG_USART_ID);
  }
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void uart_shell_init(void)
{
  BaseType_t task_create_status;
  GPIO_InitTypeDef gpio_init = {0};

  // Initialize the RX ring buffer
  ringbuf_init(&Rx_RingBuf_Handle, rx_buf, sizeof(uint8_t), UART_SHELL_RX_BUF_SIZE);

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

  // Enable UART RX interrupts
  LL_USART_EnableIT_RXNE(UART_SHELL_CFG_USART_ID);
  HAL_NVIC_SetPriority(UART_SHELL_IRQ_N, 6, 1);
  HAL_NVIC_EnableIRQ(UART_SHELL_IRQ_N);

  /* Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed. */
  setvbuf(stdout, NULL, _IONBF, 0);

  // Create the shell task
  task_create_status = xTaskCreate(uart_shell_task_handler,
                      "UART Shell task",
                      200,
                      NULL,
                      2,
                      &UART_Shell_Task_Handle);

  configASSERT(task_create_status == pdPASS);
}

UART_SHELL_REG_CMD_STATUS_T uart_shell_register_cmd(
                              char *id,
                              char *help_str,
                              UART_Shell_Cmd_Callback callback)
{
  // Make sure there is room for a new command
  if (uart_shell_num_cmds >= UART_SHELL_CMD_MAX_NUM_CMDS)
  {
    return UART_SHELL_REG_CMD_ERROR_MAX_CMDS;
  }

  // Make sure all expected data is provided
  if (id == NULL || help_str == NULL || callback == NULL)
  {
    return UART_SHELL_REG_CMD_ERROR_NULL_PTR;
  }

  // Make sure there is enough space for all the data
  if ((strlen(id) >= UART_SHELL_CMD_ID_SIZE) || (strlen(help_str) >= UART_SHELL_CMD_HELP_STR_SIZE))
  {
    return UART_SHELL_REG_CMD_ERROR_BUFFER_OVERFLOW;
  }

  // Make sure the command is not a duplicate
  for (int i = 0; i < uart_shell_num_cmds; i++)
  {
    if (strcmp(uart_shell_cmds[i].cmd_id, id) == 0)
    {
      return UART_SHELL_REG_CMD_ERROR_DUPLICATE_CMD;
    }
  }

  // Build the command struct
  UART_Shell_Command_t cmd;
  strcpy(cmd.cmd_id, id);
  strcpy(cmd.cmd_help_str, help_str);
  cmd.cmd_callback = callback;

  // Copy into the command registry and increment number of commands
  memcpy(&uart_shell_cmds[uart_shell_num_cmds], &cmd, sizeof(UART_Shell_Command_t));
  uart_shell_num_cmds++;

  return UART_SHELL_REG_CMD_OK;
}

/************************************
 * INTERRUPT HANDLERS
 ************************************/
void UART_SHELL_IRQ_Handler(void)
{
  // Check for RXNE
  if (LL_USART_IsActiveFlag_RXNE(UART_SHELL_CFG_USART_ID))
  {
    static uint8_t rx = 0x00;
    static RingBuf_Status_t status = RINGBUF_STATUS_OK;

    // Clear the interrupt flag
    LL_USART_ClearFlag_RXNE(UART_SHELL_CFG_USART_ID);

    // Read the received character
    rx = LL_USART_ReceiveData8(UART_SHELL_CFG_USART_ID);

    // Carriage return character = complete command received
    if (rx == '\r'){
      xTaskNotifyFromISR(UART_Shell_Task_Handle, UART_SHELL_SIGNAL_RX_COMPLETE, eSetBits, NULL);
      return;
    }

    // Put received byte into the ringbuffer
    status = ringbuf_put(&Rx_RingBuf_Handle, &rx);

    // Notify UART shell task of buffer overflows
    if (status == RINGBUF_STATUS_FULL)
    {
      xTaskNotifyFromISR(UART_Shell_Task_Handle, UART_SHELL_SIGNAL_RX_BUFFER_OVERFLOW, eSetBits, NULL);
      return;
    }
  }
}

/************************************
 * SYSTEM CALLS
 ************************************/
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

int _fstat(int fd, struct stat* st) {
  if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
    st->st_mode = S_IFCHR;
    return 0;
  }

  errno = EBADF;
  return 0;
}
