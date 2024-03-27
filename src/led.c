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

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

/************************************
 * EXTERN VARIABLES
 ************************************/

/************************************
 * PRIVATE MACROS AND DEFINES
 ************************************/
// LED indexes
enum LED_INDEX_T {
  LED_GREEN_INDEX,
  LED_ORANGE_INDEX,
  LED_RED_INDEX,
  LED_BLUE_INDEX,
  NUM_LED_INDEX
} LED_INDEX_T;

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

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * STATIC VARIABLES
 ************************************/
// LED lookup table
static const uint16_t LED_LOOKUP[NUM_LED_INDEX] = {LED_GREEN, LED_ORANGE, LED_RED, LED_BLUE};

// Timers used for LED toggle functions
TimerHandle_t LED_Timers[NUM_LED_INDEX];

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void led_gpio_init(uint16_t pin);
static void led_timer_callback(TimerHandle_t xTimer);

/************************************
 * STATIC FUNCTIONS
 ************************************/
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
}
