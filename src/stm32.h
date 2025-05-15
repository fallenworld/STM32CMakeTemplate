/*
 * STM32 helper functions header.
 */

#ifndef __STM32_H__
#define __STM32_H__

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#ifndef __PROJECT_NAME__
#define __PROJECT_NAME__ "STM32"
#endif

#define GPIO_Pin_0_SOURCE  GPIO_PinSource0
#define GPIO_Pin_1_SOURCE  GPIO_PinSource1
#define GPIO_Pin_2_SOURCE  GPIO_PinSource2
#define GPIO_Pin_3_SOURCE  GPIO_PinSource3
#define GPIO_Pin_4_SOURCE  GPIO_PinSource4
#define GPIO_Pin_5_SOURCE  GPIO_PinSource5
#define GPIO_Pin_6_SOURCE  GPIO_PinSource6
#define GPIO_Pin_7_SOURCE  GPIO_PinSource7
#define GPIO_Pin_8_SOURCE  GPIO_PinSource8
#define GPIO_Pin_9_SOURCE  GPIO_PinSource9
#define GPIO_Pin_10_SOURCE GPIO_PinSource10
#define GPIO_Pin_11_SOURCE GPIO_PinSource11
#define GPIO_Pin_12_SOURCE GPIO_PinSource12
#define GPIO_Pin_13_SOURCE GPIO_PinSource13
#define GPIO_Pin_14_SOURCE GPIO_PinSource14
#define GPIO_Pin_15_SOURCE GPIO_PinSource15

#define GPIO_Pin_0_EXTI_LINE  EXTI_Line0
#define GPIO_Pin_1_EXTI_LINE  EXTI_Line1
#define GPIO_Pin_2_EXTI_LINE  EXTI_Line2
#define GPIO_Pin_3_EXTI_LINE  EXTI_Line3
#define GPIO_Pin_4_EXTI_LINE  EXTI_Line4
#define GPIO_Pin_5_EXTI_LINE  EXTI_Line5
#define GPIO_Pin_6_EXTI_LINE  EXTI_Line6
#define GPIO_Pin_7_EXTI_LINE  EXTI_Line7
#define GPIO_Pin_8_EXTI_LINE  EXTI_Line8
#define GPIO_Pin_9_EXTI_LINE  EXTI_Line9
#define GPIO_Pin_10_EXTI_LINE EXTI_Line10
#define GPIO_Pin_11_EXTI_LINE EXTI_Line11
#define GPIO_Pin_12_EXTI_LINE EXTI_Line12
#define GPIO_Pin_13_EXTI_LINE EXTI_Line13
#define GPIO_Pin_14_EXTI_LINE EXTI_Line14
#define GPIO_Pin_15_EXTI_LINE EXTI_Line15

#define GPIO_Pin_0_EXTI_IRQN  EXTI0_IRQn
#define GPIO_Pin_1_EXTI_IRQN  EXTI1_IRQn
#define GPIO_Pin_2_EXTI_IRQN  EXTI2_IRQn
#define GPIO_Pin_3_EXTI_IRQN  EXTI3_IRQn
#define GPIO_Pin_4_EXTI_IRQN  EXTI4_IRQn
#define GPIO_Pin_5_EXTI_IRQN  EXTI9_5_IRQn
#define GPIO_Pin_6_EXTI_IRQN  EXTI9_5_IRQn
#define GPIO_Pin_7_EXTI_IRQN  EXTI9_5_IRQn
#define GPIO_Pin_8_EXTI_IRQN  EXTI9_5_IRQn
#define GPIO_Pin_9_EXTI_IRQN  EXTI9_5_IRQn
#define GPIO_Pin_10_EXTI_IRQN EXTI15_10_IRQn
#define GPIO_Pin_11_EXTI_IRQN EXTI15_10_IRQn
#define GPIO_Pin_12_EXTI_IRQN EXTI15_10_IRQn
#define GPIO_Pin_13_EXTI_IRQN EXTI15_10_IRQn
#define GPIO_Pin_14_EXTI_IRQN EXTI15_10_IRQn
#define GPIO_Pin_15_EXTI_IRQN EXTI15_10_IRQn

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

/* GPIO. */
bool gpio_init(GPIO_TypeDef *gpio, uint16_t pins, GPIOMode_TypeDef mode);

/* EXTI. */
void exti_init(uint8_t port_source, uint32_t exti_line, uint8_t pin_source, uint8_t pin_exti_irqn,
        EXTITrigger_TypeDef trigger, uint8_t preemption_pri, uint8_t sub_pri);

#define EXTI_INIT(gpio, pin, trigger, preemption_pri, sub_pri) do                 \
{                                                                                 \
    DEBUG_TRACE(__FILE_NAME__, __LINE__, "EXTI_INIT",                             \
            "gpio %s, pins %s, trigger %s, preemption_pri %u, sub_pri %u.\n",     \
            #gpio, #pin, #trigger, preemption_pri, sub_pri);                      \
    gpio_init(gpio, pin, GPIO_Mode_IPU);                                          \
    exti_init(gpio##_PORT_SOURCE, pin##_EXTI_LINE, pin##_SOURCE, pin##_EXTI_IRQN, \
            trigger, preemption_pri, sub_pri);                                    \
} while (0)

/* USART. */
bool usart_init(USART_TypeDef *usart);
void usart_send_byte(USART_TypeDef *usart, uint8_t byte);
void usart_send_str(USART_TypeDef *usart, const char *str);
bool usart_has_data(USART_TypeDef *usart);
uint8_t usart_wait_byte(USART_TypeDef *usart);

/* Timer. */
void timer_update_init(TIM_TypeDef *timer, bool internal_clock, uint16_t prescaler, uint16_t period);
void pwm_init(TIM_TypeDef *timer, int channel, uint32_t frequency, uint16_t period);
void pwm_set_pulse(TIM_TypeDef *timer, int channel, uint16_t pulse);
void servo_init(TIM_TypeDef *timer, int channel);
void servo_set_angle(TIM_TypeDef *timer, int channel, uint32_t angle);

/* Delay. */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/* Debug. */
bool debug_init(USART_TypeDef *usart, GPIO_TypeDef *debug_led_gpio, uint16_t debug_led_pin);
void debug_trace(const char *file, int line, const char *func, const char *format, ...);
void __assert_func(const char *file, int line, const char *func, const char *expr);
void assert_failed(uint8_t *file, uint32_t line);

#ifdef DEBUG
#define TRACE(format, ...) debug_trace(__FILE_NAME__, __LINE__, __func__, format, ##__VA_ARGS__);
#else
#define TRACE
#endif /* DEBUG */

#endif /* __STM32_H__ */
