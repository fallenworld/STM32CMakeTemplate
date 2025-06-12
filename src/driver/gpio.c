/*
 * STM32 GPIO helper functions.
 */

#include "stm32.h"

static const GPIO_TypeDef *gpio_list[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG};

static const enum IRQn pin_irq_channels[] =
{
    EXTI0_IRQn,     EXTI1_IRQn,     EXTI2_IRQn,     EXTI3_IRQn,     EXTI4_IRQn,
    EXTI9_5_IRQn,   EXTI9_5_IRQn,   EXTI9_5_IRQn,   EXTI9_5_IRQn,   EXTI9_5_IRQn,
    EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
};

static uint8_t gpio_get_index(GPIO_TypeDef *gpio)
{
    uint8_t index;
    for (index = 0; index < ARRAY_SIZE(gpio_list); ++index)
    {
        if (gpio_list[index] == gpio)
            return index;
    }
    return (uint8_t)-1;
}

static const char *gpio_name(GPIO_TypeDef *gpio)
{
    return debug_buffer_sprintf("P%c", 'A' + gpio_get_index(gpio));
}

static uint32_t gpio_port_source(GPIO_TypeDef *gpio)
{
    return gpio_get_index(gpio) - GPIO_PortSourceGPIOA;
}

static uint32_t gpio_periph(GPIO_TypeDef *gpio)
{
    return RCC_APB2Periph_GPIOA << gpio_get_index(gpio);
}

static uint8_t pin_get_index(uint16_t pin)
{
    return count_trailing_zeros(pin);
}

static const char *pins_str(uint16_t pins)
{
    char buffer[64], *ptr = buffer;

    if (pins == GPIO_Pin_All)
    {
        return "-All";
    }

    memset(buffer, 0, sizeof(buffer));
    while (pins)
    {
        int index = pin_get_index(pins);
        if (ptr == buffer)
            ptr += sprintf(ptr, "%u", index);
        else
            ptr += sprintf(ptr, ",%u", index);
        pins &= ~(1 << index);
    }

    return debug_buffer_sprintf("%s", buffer);
}

static uint8_t pin_source(uint16_t pin)
{
    return pin_get_index(pin) - GPIO_PinSource0;
}

static uint32_t pin_exti_line(uint16_t pin)
{
    return pin;
}

bool gpio_init(GPIO_TypeDef *gpio, uint16_t pins, GPIOMode_TypeDef mode)
{
    uint8_t gpio_index = gpio_get_index(gpio);
    GPIO_InitTypeDef gpio_init_def;

    if (gpio_index >= ARRAY_SIZE(gpio_list))
    {
        TRACE("Invalid gpio %p.\n", gpio);
        return false;
    }

    abp2_periph_enable(gpio_periph(gpio));

    gpio_init_def.GPIO_Mode = mode;
    gpio_init_def.GPIO_Pin = pins;
    gpio_init_def.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &gpio_init_def);

    TRACE("Inited %s%s.\n", gpio_name(gpio), pins_str(pins));

    return true;
}

bool gpio_pin_init(const struct gpio_pin *pin, GPIOMode_TypeDef mode)
{
    return gpio_init(pin->gpio, pin->pin, mode);
}

uint8_t gpio_pin_read(const struct gpio_pin *pin)
{
    return GPIO_ReadInputDataBit(pin->gpio, pin->pin);
}

void gpio_pin_write(const struct gpio_pin *pin, uint8_t bit)
{
    GPIO_WriteBit(pin->gpio, pin->pin, bit ? Bit_SET : Bit_RESET);
}

bool exti_init(GPIO_TypeDef *gpio, uint16_t pin, irq_handler handler,
        EXTITrigger_TypeDef trigger, uint8_t preemption_pri, uint8_t sub_pri)
{
    uint8_t gpio_index = gpio_get_index(gpio), pin_index = pin_get_index(pin);
    EXTI_InitTypeDef exti_init_def;
    NVIC_InitTypeDef nvic_init_def;

    if (gpio_index >= ARRAY_SIZE(gpio_list) || pin_index >= 16)
    {
        TRACE("Invalid parameter: gpio %p, pin %u.\n", gpio, pin);
        return false;
    }

    if (!gpio_init(gpio, pin, GPIO_Mode_IPU))
    {
        TRACE("Failed to init %s%u.\n", gpio_name(gpio), pin_index);
        return false;
    }

    if (!exti_set_handler(pin_exti_line(pin), handler))
    {
        TRACE("Failed to set EXTI handler to %p.\n", handler);
        return false;
    }

    abp2_periph_enable(RCC_APB2Periph_AFIO);

    GPIO_EXTILineConfig(gpio_port_source(gpio), pin_source(pin));

    exti_init_def.EXTI_Line = pin_exti_line(pin);
    exti_init_def.EXTI_Trigger = trigger;
    exti_init_def.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_def.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_def);

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic_init_def.NVIC_IRQChannel = pin_irq_channels[pin_index];
    nvic_init_def.NVIC_IRQChannelPreemptionPriority = preemption_pri;
    nvic_init_def.NVIC_IRQChannelSubPriority = sub_pri;
    nvic_init_def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_def);

    TRACE("Inited EXTI on %s%u.\n", gpio_name(gpio), pin_index);

    return true;
}
