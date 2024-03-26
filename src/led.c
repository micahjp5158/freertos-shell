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

/************************************
 * PRIVATE TYPEDEFS
 ************************************/

/************************************
 * STATIC VARIABLES
 ************************************/

/************************************
 * GLOBAL VARIABLES
 ************************************/

/************************************
 * STATIC FUNCTION PROTOTYPES
 ************************************/
static void led_gpio_init(uint16_t pin);

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

/************************************
 * GLOBAL FUNCTIONS
 ************************************/
void led_init(void)
{
  // Initialize LED GPIO clock
  LED_CLK_ENABLE();

  // Initialize each LED pin
  led_gpio_init(LED_GREEN);
  led_gpio_init(LED_ORANGE);
  led_gpio_init(LED_RED);
  led_gpio_init(LED_BLUE);
}
