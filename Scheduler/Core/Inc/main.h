#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* --------------------------------------------------------------------------
 * Lab 4 pin mapping from the user's Proteus schematic.
 * PA1..PA5 : five periodic LEDs (D1..D5)
 * PA6      : button indicator LED (D6)
 * PB0      : push button, active HIGH, external pull-down resistor
 * PA9      : USART1_TX -> Virtual Terminal RXD
 * PA10     : USART1_RX is not used by this transmit-only application.
 * -------------------------------------------------------------------------- */
#define LED_D1_PIN              GPIO_PIN_1
#define LED_D2_PIN              GPIO_PIN_2
#define LED_D3_PIN              GPIO_PIN_3
#define LED_D4_PIN              GPIO_PIN_4
#define LED_D5_PIN              GPIO_PIN_5
#define LED_D6_PIN              GPIO_PIN_6
#define LED_GPIO_PORT           GPIOA

#define BUTTON_PIN              GPIO_PIN_0
#define BUTTON_GPIO_PORT        GPIOB

#define UART_TX_PIN             GPIO_PIN_9
#define UART_GPIO_PORT          GPIOA

/* The user's LED wiring is active LOW: RESET = LED ON, SET = LED OFF. */
#define LED_ACTIVE_LOW          1U
#if (LED_ACTIVE_LOW != 0U)
#define LED_ON_STATE            GPIO_PIN_RESET
#define LED_OFF_STATE           GPIO_PIN_SET
#else
#define LED_ON_STATE            GPIO_PIN_SET
#define LED_OFF_STATE           GPIO_PIN_RESET
#endif

void Error_Handler(void);
void UART_Tx_Empty_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
