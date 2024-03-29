/**
 ********************************************************************************
 * @file    led.c
 * @author  micahjp5158
 * @date    3-27-2024
 * @brief   Handles LED tasks.
 ********************************************************************************
 */

/************************************
 * INCLUDES
 ************************************/
#include "led.h"

#include "uart_shell.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
// LED defines
#define LED_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()

#define LED_GPIO_PORT     GPIOD
#define LED_GREEN_PIN     GPIO_PIN_12
#define LED_ORANGE_PIN    GPIO_PIN_13
#define LED_RED_PIN       GPIO_PIN_14
#define LED_BLUE_PIN      GPIO_PIN_15

#define LED_GPIO_MODE     GPIO_MODE_OUTPUT_PP
#define LED_GPIO_PULL     GPIO_PULLUP
#define LED_GPIO_SPEED    GPIO_SPEED_FREQ_LOW

#define LED_BLINK_FAST_MS 250
#define LED_BLINK_SLOW_MS 1000

// LED command queue size
#define LED_CMD_QUEUE_SIZE  8

// LED command strings
#define LED_CMD_ID_STR    "led"
#define LED_CMD_INFO_STR  "Control development board LEDs"
#define LED_CMD_ERROR_STR "led: Error parsing command. Enter 'led help' for usage.\n"
#define LED_CMD_HELP_STR  "\
LED Usage: led [color] [mode] [speed]\n\
  [color]\n\
    LED color selection\n\
    Options: green, orange, red, blue\n\
  [mode]\n\
    LED mode selection\n\
    Options: off, on, blink\n\
  [speed]\n\
    LED blink speed. Only required when [mode] = blink.\n\
    Options: fast, slow\n"

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
// LEDs
typedef enum LED {
  LED_GREEN,
  LED_ORANGE,
  LED_RED,
  LED_BLUE,
  NUM_LED
} LED_T;

// LED operating modes
typedef enum LED_MODE {
  LED_MODE_OFF,
  LED_MODE_ON,
  LED_MODE_BLINK_SLOW,
  LED_MODE_BLINK_FAST,
  NUM_LED_MODE
} LED_MODE_T;

// LED command
typedef struct LED_Command {
  LED_T led;
  LED_MODE_T mode;
} LED_Command_t;

/************************************
 * STATIC VARIABLES
 ************************************/
// LED lookup table
static const uint16_t LED_LOOKUP[NUM_LED] = {LED_GREEN_PIN, LED_ORANGE_PIN, LED_RED_PIN, LED_BLUE_PIN};

// Timers used for LED toggle functions
static TimerHandle_t LED_Timers[NUM_LED];

// Queue for LED commands
static QueueHandle_t LED_Command_Queue;

// Task to process LED commands
static TaskHandle_t LED_Command_Task_Handle;

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void led_process_cmd_callback(char *cmd);
static void led_cmd_task_handler(void *parameters);
static void led_gpio_init(uint16_t pin);
static void led_timer_callback(TimerHandle_t xTimer);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void led_process_cmd_callback(char *cmd_str)
{
  LED_Command_t cmd;
  char *token;

  // Grab next token
  token = strtok(cmd_str, " ");

  // Make sure the token exists
  if (token == NULL)
  {
    printf(LED_CMD_ERROR_STR);
    return;
  }

  // Check for help command
  if (strcmp(token, "help") == 0) {
    printf(LED_CMD_HELP_STR);
    return;
  }

  // Otherwise, check for LED color
  if (strcmp(token, "green") == 0) {
    cmd.led = LED_GREEN;
  } else if (strcmp(token, "orange") == 0) {
    cmd.led = LED_ORANGE;
  } else if (strcmp(token, "red") == 0) {
    cmd.led = LED_RED;
  } else if (strcmp(token, "blue") == 0) {
    cmd.led = LED_BLUE;
  } else {
    printf(LED_CMD_ERROR_STR);
    return;
  }

  // Grab next token
  token = strtok(NULL, " ");

  // Make sure the token exists
  if (token == NULL)
  {
    printf(LED_CMD_ERROR_STR);
    return;
  }

  // Check for LED mode
  if (strcmp(token, "off") == 0) {
    cmd.mode = LED_MODE_OFF;
  } else if (strcmp(token, "on") == 0) {
    cmd.mode = LED_MODE_ON;
  } else if (strcmp(token, "blink") == 0) {
    // Should be one more token for speed
    token = strtok(NULL, " ");
    if (strcmp(token, "fast") == 0) {
      cmd.mode = LED_MODE_BLINK_FAST;
    } else if (strcmp(token, "slow") == 0) {
      cmd.mode = LED_MODE_BLINK_SLOW;
    } else {
      printf(LED_CMD_ERROR_STR);
      return;
    }
  } else {
    printf(LED_CMD_ERROR_STR);
    return;
  }

  // Put the command into the command queue
  xQueueSendToBack(LED_Command_Queue, &cmd, portMAX_DELAY);
}

static void led_cmd_task_handler(void *parameters)
{
  LED_Command_t cmd;

  while(1)
  {
    xQueueReceive(LED_Command_Queue, (void *)&cmd, portMAX_DELAY);
    switch(cmd.mode)
    {
      case LED_MODE_OFF:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.led], GPIO_PIN_RESET);
        xTimerChangePeriod(LED_Timers[cmd.led], portMAX_DELAY, portMAX_DELAY);
        break;
      case LED_MODE_ON:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.led], GPIO_PIN_SET);
        xTimerChangePeriod(LED_Timers[cmd.led], portMAX_DELAY, portMAX_DELAY);
        break;
      case LED_MODE_BLINK_FAST:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.led], GPIO_PIN_SET);
        xTimerChangePeriod(LED_Timers[cmd.led], LED_BLINK_FAST_MS, portMAX_DELAY);
        break;
      case LED_MODE_BLINK_SLOW:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.led], GPIO_PIN_SET);
        xTimerChangePeriod(LED_Timers[cmd.led], LED_BLINK_SLOW_MS, portMAX_DELAY);
        break;
      default:
        break;
    }
  }
}

static void led_gpio_init(uint16_t pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = LED_GPIO_MODE;;
  GPIO_InitStruct.Pull = LED_GPIO_PULL;
  GPIO_InitStruct.Speed = LED_GPIO_SPEED;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_GPIO_PORT, pin, GPIO_PIN_RESET);
}

static void led_timer_callback(TimerHandle_t xTimer)
{
  // Select LED based on timer ID
  uint32_t id = (uint32_t) pvTimerGetTimerID(xTimer);
  if (id >= NUM_LED)
  {
    // TODO error handling
    return;
  }

  // Toggle the selected pin
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_LOOKUP[id]);
}

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void led_init(void)
{
  BaseType_t task_create_status;

  // Initialize LED GPIO clock
  LED_CLK_ENABLE();

  // Initialize each LED pin
  for (int i = 0; i < NUM_LED; i++)
  {
    led_gpio_init(LED_LOOKUP[i]);
  }

  // Set up LED software timers
  LED_Timers[LED_GREEN] = xTimerCreate("Green LED timer", portMAX_DELAY, pdTRUE, (void *)LED_GREEN, led_timer_callback);
  LED_Timers[LED_ORANGE] = xTimerCreate("Orange LED timer", portMAX_DELAY, pdTRUE, (void *)LED_ORANGE, led_timer_callback);
  LED_Timers[LED_RED] = xTimerCreate("Red LED timer", portMAX_DELAY, pdTRUE, (void *)LED_RED, led_timer_callback);
  LED_Timers[LED_BLUE] = xTimerCreate("Blue LED timer", portMAX_DELAY, pdTRUE, (void *)LED_BLUE, led_timer_callback);

  // Start the LED software timers
  for (int i = 0; i < NUM_LED; i++)
  {
    xTimerStart(LED_Timers[i], portMAX_DELAY);
  }

  // Create the LED command queue
  LED_Command_Queue = xQueueCreate(LED_CMD_QUEUE_SIZE, sizeof(LED_Command_t));

  // Create the LED command processing task
  task_create_status = xTaskCreate(led_cmd_task_handler,
                      "LED command processing task",
                      200,
                      NULL,
                      2,
                      &LED_Command_Task_Handle);

  configASSERT(task_create_status == pdPASS);

  // Register the LED shell command
  UART_SHELL_REG_CMD_STATUS_T status = uart_shell_register_cmd("led", "Control development board LEDs", led_process_cmd_callback);
  (void) status;
}
