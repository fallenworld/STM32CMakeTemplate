/*
 * GPIO functions.
 */

#include "stm32.h"

void gpio_init(GPIO_TypeDef *gpio_x, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio_init_def;
    uint32_t rcc_apb2_periph;

    if (gpio_x == GPIOA)
        rcc_apb2_periph = RCC_APB2Periph_GPIOA;
    else if (gpio_x == GPIOB)
        rcc_apb2_periph = RCC_APB2Periph_GPIOB;
    else if (gpio_x == GPIOC)
        rcc_apb2_periph = RCC_APB2Periph_GPIOC;
    else if (gpio_x == GPIOD)
        rcc_apb2_periph = RCC_APB2Periph_GPIOD;
    else if (gpio_x == GPIOE)
        rcc_apb2_periph = RCC_APB2Periph_GPIOE;
    else if (gpio_x == GPIOF)
        rcc_apb2_periph = RCC_APB2Periph_GPIOF;
    else if (gpio_x == GPIOG)
        rcc_apb2_periph = RCC_APB2Periph_GPIOG;
    else
        return;

    gpio_init_def.GPIO_Mode = mode;
    gpio_init_def.GPIO_Pin = pins;
    gpio_init_def.GPIO_Speed = GPIO_Speed_50MHz;

    RCC_APB2PeriphClockCmd(rcc_apb2_periph, ENABLE);
    GPIO_Init(gpio_x, &gpio_init_def);
}
