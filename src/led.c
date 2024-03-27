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

#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
// LED defines
#define LED_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()

#define LED_GPIO_PORT     GPIOD
#define LED_GREEN         GPIO_PIN_12
#define LED_ORANGE        GPIO_PIN_13
#define LED_RED           GPIO_PIN_14
#define LED_BLUE          GPIO_PIN_15

#define LED_GPIO_MODE     GPIO_MODE_OUTPUT_PP
#define LED_GPIO_PULL     GPIO_PULLUP
#define LED_GPIO_SPEED    GPIO_SPEED_FREQ_LOW

#define LED_BLINK_FAST_MS 250
#define LED_BLINK_SLOW_MS 1000

// LED Command queue size
#define LED_CMD_QUEUE_SIZE  8

/************************************
 * PRIVATE TYPEDEFS
 ************************************/
// LED indexes
typedef enum LED_INDEX {
  LED_GREEN_INDEX,
  LED_ORANGE_INDEX,
  LED_RED_INDEX,
  LED_BLUE_INDEX,
  NUM_LED_INDEX
} LED_INDEX_T;

typedef enum LED_MODE {
  LED_MODE_OFF,
  LED_MODE_ON,
  LED_MODE_BLINK_SLOW,
  LED_MODE_BLINK_FAST,
  NUM_LED_MODE
} LED_MODE_T;

typedef struct LED_CMD {
  LED_INDEX_T idx;
  LED_MODE_T mode;
} LED_CMD_T;

/************************************
 * STATIC VARIABLES
 ************************************/
// LED lookup table
static const uint16_t LED_LOOKUP[NUM_LED_INDEX] = {LED_GREEN, LED_ORANGE, LED_RED, LED_BLUE};

// Timers used for LED toggle functions
static TimerHandle_t LED_Timers[NUM_LED_INDEX];

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
static void led_cmd_task_handler(void *parameters);
static void led_gpio_init(uint16_t pin);
static void led_timer_callback(TimerHandle_t xTimer);

/************************************
 * STATIC FUNCTIONS
 ************************************/
static void led_cmd_task_handler(void *parameters)
{
  LED_CMD_T cmd;

  while(1)
  {
    xQueueReceive(LED_Command_Queue, (void *)&cmd, portMAX_DELAY);
    switch(cmd.mode)
    {
      case LED_MODE_OFF:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.idx], GPIO_PIN_RESET);
        xTimerChangePeriod(LED_Timers[cmd.idx], portMAX_DELAY, portMAX_DELAY);
        break;
      case LED_MODE_ON:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.idx], GPIO_PIN_SET);
        xTimerChangePeriod(LED_Timers[cmd.idx], portMAX_DELAY, portMAX_DELAY);
        break;
      case LED_MODE_BLINK_FAST:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.idx], GPIO_PIN_SET);
        xTimerChangePeriod(LED_Timers[cmd.idx], LED_BLINK_FAST_MS, portMAX_DELAY);
        break;
      case LED_MODE_BLINK_SLOW:
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_LOOKUP[cmd.idx], GPIO_PIN_SET);
        xTimerChangePeriod(LED_Timers[cmd.idx], LED_BLINK_SLOW_MS, portMAX_DELAY);
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
  if (id >= NUM_LED_INDEX)
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
  for (int i = 0; i < NUM_LED_INDEX; i++)
  {
    led_gpio_init(LED_LOOKUP[i]);
  }

  // Set up LED software timers
  LED_Timers[LED_GREEN_INDEX] = xTimerCreate("Green LED timer", portMAX_DELAY, pdTRUE, (void *)LED_GREEN_INDEX, led_timer_callback);
  LED_Timers[LED_ORANGE_INDEX] = xTimerCreate("Orange LED timer", portMAX_DELAY, pdTRUE, (void *)LED_ORANGE_INDEX, led_timer_callback);
  LED_Timers[LED_RED_INDEX] = xTimerCreate("Red LED timer", portMAX_DELAY, pdTRUE, (void *)LED_RED_INDEX, led_timer_callback);
  LED_Timers[LED_BLUE_INDEX] = xTimerCreate("Blue LED timer", portMAX_DELAY, pdTRUE, (void *)LED_BLUE_INDEX, led_timer_callback);

  // Start the LED software timers
  for (int i = 0; i < NUM_LED_INDEX; i++)
  {
    xTimerStart(LED_Timers[i], portMAX_DELAY);
  }

  // Create the LED command queue
  LED_Command_Queue = xQueueCreate(LED_CMD_QUEUE_SIZE, sizeof(LED_CMD_T));

  // Create the LED command processing task
  task_create_status = xTaskCreate(led_cmd_task_handler,
                      "LED command processing task",
                      200,
                      NULL,
                      2,
                      &LED_Command_Task_Handle);

  configASSERT(task_create_status == pdPASS);
}
