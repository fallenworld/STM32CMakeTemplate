/*
 * STM32 helper functions header.
 */

#ifndef __STM32_H__
#define __STM32_H__

#include <stdarg.h>
#include <stdio.h>
#include "stm32f10x.h"

/* delay.c */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/* gpio.c */
void gpio_init(GPIO_TypeDef *gpiox, uint16_t pins, GPIOMode_TypeDef mode);

/* usart.c */
void usart_init(USART_TypeDef *usartx);
void usart_send_byte(USART_TypeDef *usart_x, uint8_t byte);
void usart_send_str(USART_TypeDef *usart_x, const char *str);

/* debug.c */
void debug_init(USART_TypeDef *usart_x);
void vprint(const char *format, va_list args);
void print(const char *format, ...);
void debug_trace(const char *file, const char *func, int line, const char *format, ...);
#define trace(format, ...) \
        debug_trace(__FILE_NAME__, __func__, __LINE__, format, ##__VA_ARGS__);

#endif /* __STM32_H__ */
