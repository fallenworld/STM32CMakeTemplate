/*
 * STM32 GPIO helper functions.
 */

#include "stm32.h"

struct gpio_info
{
    GPIO_TypeDef *gpio;
    const char *name;
    uint32_t periph;
    uint8_t port_source;
};

static const struct gpio_info gpio_info_list[] =
{
    {GPIOA, "PA", RCC_APB2Periph_GPIOA, GPIO_PortSourceGPIOA},
    {GPIOB, "PB", RCC_APB2Periph_GPIOB, GPIO_PortSourceGPIOB},
    {GPIOC, "PC", RCC_APB2Periph_GPIOC, GPIO_PortSourceGPIOC},
    {GPIOD, "PD", RCC_APB2Periph_GPIOD, GPIO_PortSourceGPIOD},
    {GPIOE, "PE", RCC_APB2Periph_GPIOE, GPIO_PortSourceGPIOE},
    {GPIOF, "PF", RCC_APB2Periph_GPIOF, GPIO_PortSourceGPIOF},
    {GPIOG, "PG", RCC_APB2Periph_GPIOG, GPIO_PortSourceGPIOG},
};

static const struct gpio_info *gpio_info_find(const GPIO_TypeDef *gpio)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(gpio_info_list); ++i)
    {
        if (gpio_info_list[i].gpio == gpio)
            return &gpio_info_list[i];
    }
    return NULL;
}

bool gpio_init(GPIO_TypeDef *gpio, uint16_t pins, GPIOMode_TypeDef mode)
{
    const struct gpio_info *gpio_info = gpio_info_find(gpio);
    GPIO_InitTypeDef gpio_init_def;

    if (!gpio_info)
    {
        TRACE("Invalid gpio %p.\n", gpio);
        return false;
    }

    abp2_periph_enable(gpio_info->periph);

    gpio_init_def.GPIO_Mode = mode;
    gpio_init_def.GPIO_Pin = pins;
    gpio_init_def.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &gpio_init_def);

    TRACE("Inited %s, pins %#x.\n", gpio_info->name, pins);

    return true;
}

void exti_init(uint8_t port_source, uint32_t exti_line, uint8_t pin_source, uint8_t pin_exti_irqn,
        EXTITrigger_TypeDef trigger, uint8_t preemption_pri, uint8_t sub_pri)
{
    EXTI_InitTypeDef exti_init_def;
    NVIC_InitTypeDef nvic_init_def;

    abp2_periph_enable(RCC_APB2Periph_AFIO);

    GPIO_EXTILineConfig(port_source, pin_source);

    exti_init_def.EXTI_Line = exti_line;
    exti_init_def.EXTI_Trigger = trigger;
    exti_init_def.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_def.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_def);

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic_init_def.NVIC_IRQChannel = pin_exti_irqn;
    nvic_init_def.NVIC_IRQChannelPreemptionPriority = preemption_pri;
    nvic_init_def.NVIC_IRQChannelSubPriority = sub_pri;
    nvic_init_def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_def);
}
