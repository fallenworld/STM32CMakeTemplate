/*
 * Helper functions.
 */

#ifndef __HELPERS_H__
#define __HELPERS_H__

#include "stm32f10x.h"

static inline uint32_t gpiox_to_apb2_periph(GPIO_TypeDef* gpiox)
{
    if (gpiox == GPIOA)
        return RCC_APB2Periph_GPIOA;
    if (gpiox == GPIOB)
        return RCC_APB2Periph_GPIOB;
    if (gpiox == GPIOC)
        return RCC_APB2Periph_GPIOC;
    if (gpiox == GPIOD)
        return RCC_APB2Periph_GPIOD;
    if (gpiox == GPIOE)
        return RCC_APB2Periph_GPIOE;
    if (gpiox == GPIOF)
        return RCC_APB2Periph_GPIOF;
    if (gpiox == GPIOG)
        return RCC_APB2Periph_GPIOG;
    return 0;
}

static inline void delay_us(uint32_t us)
{
    SysTick->LOAD = 72 * us;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    while (!(SysTick->CTRL & 0x00010000)) {}
    SysTick->CTRL = 4;
}

static inline void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}

static inline void gpio_init(GPIO_TypeDef* gpiox, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio_init_def;

    gpio_init_def.GPIO_Mode = mode;
    gpio_init_def.GPIO_Pin = pins;
    gpio_init_def.GPIO_Speed = GPIO_Speed_50MHz;

    RCC_APB2PeriphClockCmd(gpiox_to_apb2_periph(gpiox), ENABLE);
    GPIO_Init(GPIOA, &gpio_init_def);
}

#endif /* __HELPERS_H__ */
