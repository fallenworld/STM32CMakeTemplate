/*
 * STM32 helper functions header.
 */

#ifndef __STM32_H__
#define __STM32_H__

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32f10x.h"

#ifndef __PROJECT_NAME__
#define __PROJECT_NAME__ "STM32"
#endif

#define GPIOA_PERIPH RCC_APB2Periph_GPIOA
#define GPIOB_PERIPH RCC_APB2Periph_GPIOB
#define GPIOC_PERIPH RCC_APB2Periph_GPIOC
#define GPIOD_PERIPH RCC_APB2Periph_GPIOD
#define GPIOE_PERIPH RCC_APB2Periph_GPIOE
#define GPIOF_PERIPH RCC_APB2Periph_GPIOF
#define GPIOG_PERIPH RCC_APB2Periph_GPIOG

#define GPIOA_PORT_SOURCE GPIO_PortSourceGPIOA
#define GPIOB_PORT_SOURCE GPIO_PortSourceGPIOB
#define GPIOC_PORT_SOURCE GPIO_PortSourceGPIOC
#define GPIOD_PORT_SOURCE GPIO_PortSourceGPIOD
#define GPIOE_PORT_SOURCE GPIO_PortSourceGPIOE
#define GPIOF_PORT_SOURCE GPIO_PortSourceGPIOF
#define GPIOG_PORT_SOURCE GPIO_PortSourceGPIOG

#define USART1_GPIO        GPIOA
#define USART1_GPIO_PERIPH GPIOA_PERIPH
#define USART1_PERIPH      RCC_APB2Periph_USART1
#define USART1_TX_PIN      GPIO_Pin_9
#define USART1_RX_PIN      GPIO_Pin_10

#define USART2_GPIO        GPIOA
#define USART2_GPIO_PERIPH GPIOA_PERIPH
#define USART2_PERIPH      RCC_APB1Periph_USART2
#define USART2_TX_PIN      GPIO_Pin_2
#define USART2_RX_PIN      GPIO_Pin_3

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
void gpio_init(GPIO_TypeDef *gpio, uint16_t pins, GPIOMode_TypeDef mode, uint32_t gpio_periph);

#define GPIO_INIT(gpio, pins, mode) do                                                \
{                                                                                     \
    DEBUG_TRACE(__FILE_NAME__, __LINE__, "GPIO_INIT", "gpio %s, pins %s, mode %s.\n", \
            #gpio, #pins, #mode);                                                     \
    gpio_init(gpio, pins, mode, gpio##_PERIPH);                                       \
} while (0)

/* EXTI. */
void exti_init(uint8_t port_source, uint32_t exti_line, uint8_t pin_source, uint8_t pin_exti_irqn,
        EXTITrigger_TypeDef trigger, uint8_t preemption_pri, uint8_t sub_pri);

#define EXTI_INIT(gpio, pin, trigger, preemption_pri, sub_pri) do                 \
{                                                                                 \
    DEBUG_TRACE(__FILE_NAME__, __LINE__, "EXTI_INIT",                             \
            "gpio %s, pins %s, trigger %s, preemption_pri %u, sub_pri %u.\n",     \
            #gpio, #pin, #trigger, preemption_pri, sub_pri);                      \
    gpio_init(gpio, pin, GPIO_Mode_IPU, gpio##_PERIPH);                           \
    exti_init(gpio##_PORT_SOURCE, pin##_EXTI_LINE, pin##_SOURCE, pin##_EXTI_IRQN, \
            trigger, preemption_pri, sub_pri);                                    \
} while (0)

/* USART. */
void usart_init(USART_TypeDef *usart, uint32_t usart_periph);
void usart_send_byte(USART_TypeDef *usart, uint8_t byte);
void usart_send_str(USART_TypeDef *usart, const char *str);
bool usart_has_data(USART_TypeDef *usart);
uint8_t usart_wait_byte(USART_TypeDef *usart);

#define USART_INIT(usart) do                                                       \
{                                                                                  \
    DEBUG_TRACE(__FILE_NAME__, __LINE__, "USART_INIT", "usart %s.\n", #usart);     \
    gpio_init(usart##_GPIO, usart##_TX_PIN, GPIO_Mode_AF_PP, usart##_GPIO_PERIPH); \
    gpio_init(usart##_GPIO, usart##_RX_PIN, GPIO_Mode_IPU,   usart##_GPIO_PERIPH); \
    usart_init(usart, usart##_PERIPH);                                             \
} while (0)

void timer_init();

/* Delay. */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/* Debug. */
void debug_set_usart(USART_TypeDef *usart);
void vprint(const char *format, va_list args);
void print(const char *format, ...);
void debug_trace(const char *file, int line, const char *func, const char *format, ...);

#ifdef DEBUG
#define DEBUG_TRACE debug_trace
#define DEBUG_INIT(usart) do                                                       \
{                                                                                  \
    gpio_init(usart##_GPIO, usart##_TX_PIN, GPIO_Mode_AF_PP, usart##_GPIO_PERIPH); \
    gpio_init(usart##_GPIO, usart##_RX_PIN, GPIO_Mode_IPU,   usart##_GPIO_PERIPH); \
    usart_init(usart, usart##_PERIPH);                                             \
    debug_set_usart(usart);                                                        \
    DEBUG_TRACE(__FILE_NAME__, __LINE__,                                           \
            "DEBUG_INIT", "Debug output channel initialized at %s.\n", #usart);    \
} while (0)
#else
#define DEBUG_TRACE
#define DEBUG_INIT
#endif /* DEBUG */

#define TRACE(format, ...) DEBUG_TRACE(__FILE_NAME__, ___LINE__, _func__, format, ##__VA_ARGS__);

#endif /* __STM32_H__ */
