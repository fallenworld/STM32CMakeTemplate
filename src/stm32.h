/*
 * STM32 helper functions header.
 */

#ifndef __STM32_H__
#define __STM32_H__

#include "stm32f10x.h"

/* delay.c */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/* gpio.c */
void gpio_init(GPIO_TypeDef *gpiox, uint16_t pins, GPIOMode_TypeDef mode);

/* usart.c */
void usart_init(USART_TypeDef *usartx);
void usart_send_byte(USART_TypeDef *usart_x, uint8_t byte);

#endif /* __STM32_H__ */
