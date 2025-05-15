/*
 * STM32 helper functions.
 */

#include "stm32.h"

#define SYSTEM_CLOCK_HZ    (72 * 1000 * 1000)
#define SERVO_FREQUENCY    (50)
#define SERVO_PERIOD_COUNT (2000)

static USART_TypeDef *debug_usart = NULL;
static GPIO_TypeDef *debug_led_gpio = NULL;
static uint16_t debug_led_pin = 0;

struct gpio_info
{
    GPIO_TypeDef *gpio;
    const char *name;
    uint32_t periph;
    uint8_t port_source;
};

struct usart_info
{
    USART_TypeDef *usart;
    const char *name;
    uint32_t periph;
    GPIO_TypeDef *tx_gpio;
    uint16_t tx_pin;
    GPIO_TypeDef *rx_gpio;
    uint16_t rx_pin;
};

struct timer_info
{
    TIM_TypeDef *timer;
    const char *name;
    uint32_t periph;
    enum IRQn irqn;

    GPIO_TypeDef *channel1_gpio;
    uint32_t channel1_pin;
    GPIO_TypeDef *channel2_gpio;
    uint32_t channel2_pin;
    GPIO_TypeDef *channel3_gpio;
    uint32_t channel3_pin;
    GPIO_TypeDef *channel4_gpio;
    uint32_t channel4_pin;
};

static const struct gpio_info gpio_info_list[] =
{
    {GPIOA, "GPIOA", RCC_APB2Periph_GPIOA, GPIO_PortSourceGPIOA},
    {GPIOB, "GPIOB", RCC_APB2Periph_GPIOB, GPIO_PortSourceGPIOB},
    {GPIOC, "GPIOC", RCC_APB2Periph_GPIOC, GPIO_PortSourceGPIOC},
    {GPIOD, "GPIOD", RCC_APB2Periph_GPIOD, GPIO_PortSourceGPIOD},
    {GPIOE, "GPIOE", RCC_APB2Periph_GPIOE, GPIO_PortSourceGPIOE},
    {GPIOF, "GPIOF", RCC_APB2Periph_GPIOF, GPIO_PortSourceGPIOF},
    {GPIOG, "GPIOG", RCC_APB2Periph_GPIOG, GPIO_PortSourceGPIOG},
};

static const struct usart_info usart_info_list[] =
{
    {USART1, "USART1", RCC_APB2Periph_USART1, GPIOA, GPIO_Pin_9, GPIOA, GPIO_Pin_10},
    {USART2, "USART2", RCC_APB1Periph_USART2, GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3},
};

static const struct timer_info timer_info_list[] =
{
    {
        TIM2, "TIM2", RCC_APB1Periph_TIM2, TIM2_IRQn,
        GPIOA, GPIO_Pin_0,
        GPIOA, GPIO_Pin_1,
        GPIOA, GPIO_Pin_2,
        GPIOA, GPIO_Pin_3
    },
    {
        TIM3, "TIM3", RCC_APB1Periph_TIM3, TIM3_IRQn,
        GPIOA, GPIO_Pin_6,
        GPIOA, GPIO_Pin_7,
        GPIOB, GPIO_Pin_0,
        GPIOB, GPIO_Pin_1
    },
    {
        TIM4, "TIM4", RCC_APB1Periph_TIM4, TIM4_IRQn,
        GPIOB, GPIO_Pin_6,
        GPIOB, GPIO_Pin_7,
        GPIOB, GPIO_Pin_8,
        GPIOB, GPIO_Pin_9
    },
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

static const struct timer_info *timer_info_find(const TIM_TypeDef *timer)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(timer_info_list); ++i)
    {
        if (timer_info_list[i].timer == timer)
            return &timer_info_list[i];
    }
    return NULL;
}

static const struct usart_info *usart_info_find(const USART_TypeDef *usart)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(usart_info_list); ++i)
    {
        if (usart_info_list[i].usart == usart)
            return &usart_info_list[i];
    }
    return NULL;
}

bool is_abp1_periph_enabled(uint32_t periph)
{
    return !!(RCC->APB1ENR & periph);
}

bool is_abp2_periph_enabled(uint32_t periph)
{
    return !!(RCC->APB2ENR & periph);
}

void abp1_periph_enable(uint32_t periph)
{
    if (!is_abp1_periph_enabled(periph))
        RCC_APB1PeriphClockCmd(periph, ENABLE);
}

void abp2_periph_enable(uint32_t periph)
{
    if (!is_abp2_periph_enabled(periph))
        RCC_APB2PeriphClockCmd(periph, ENABLE);
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

const char *usart_name(const USART_TypeDef *usart)
{
    const struct usart_info *usart_info = usart_info_find(usart);
    if (!usart_info)
        return "(Invalid USART)";
    return usart_info->name;
}

bool usart_init(USART_TypeDef *usart)
{
    const struct usart_info *usart_info = usart_info_find(usart);
    USART_InitTypeDef usart_init_def;

    if (!usart_info)
    {
        TRACE("Invalid usart %p.\n", usart);
        return false;
    }

    if (!gpio_init(usart_info->tx_gpio, usart_info->tx_pin, GPIO_Mode_AF_PP)
            || !gpio_init(usart_info->rx_gpio, usart_info->rx_pin, GPIO_Mode_IPU))
    {
        TRACE("Failed to init GPIO.\n");
        return false;
    }

    if (usart == USART1)
        abp2_periph_enable(usart_info->periph);
    else
        abp1_periph_enable(usart_info->periph);

    USART_StructInit(&usart_init_def);
    USART_Init(usart, &usart_init_def);
    USART_Cmd(usart, ENABLE);

    TRACE("Inited %s.\n", usart_info->name);

    return true;
}

void usart_send_byte(USART_TypeDef *usart, uint8_t byte)
{
    USART_SendData(usart, byte);
    while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET) {}
}

void usart_send_str(USART_TypeDef *usart, const char *str)
{
    while (*str)
    {
        usart_send_byte(usart, *str);
        ++str;
    }
}

bool usart_has_data(USART_TypeDef *usart)
{
    return USART_GetFlagStatus(usart, USART_FLAG_RXNE) == SET;
}

uint8_t usart_wait_byte(USART_TypeDef *usart)
{
    while (!usart_has_data(usart)) {}
    return USART_ReceiveData(usart);
}

static void timer_time_base_init(const struct timer_info *timer_info,
        bool internal_clock, uint16_t prescaler, uint16_t period)
{
    TIM_TimeBaseInitTypeDef timer_init_def;

    TRACE("timer %s, internal_clock %s, precaler %u, period %u.\n",
            timer_info->name, internal_clock ? "true" : "false", prescaler, period);

    abp1_periph_enable(timer_info->periph);

    if (internal_clock)
        TIM_InternalClockConfig(timer_info->timer);
    else
        TIM_ETRClockMode2Config(timer_info->timer, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x0f);

    TIM_TimeBaseStructInit(&timer_init_def);
    timer_init_def.TIM_Prescaler = prescaler;
    timer_init_def.TIM_Period = period;
    TIM_TimeBaseInit(timer_info->timer, &timer_init_def);
}

void timer_update_init(TIM_TypeDef *timer, bool internal_clock, uint16_t prescaler, uint16_t period)
{
    const struct timer_info *timer_info = timer_info_find(timer);
    NVIC_InitTypeDef nvic_init_def;

    if (!timer_info)
    {
        TRACE("Invalid timer %p.\n", timer);
        return;
    }

    timer_time_base_init(timer_info, prescaler, period, internal_clock);

    TIM_ClearFlag(timer, TIM_FLAG_Update);
    TIM_ITConfig(timer, TIM_IT_Update, ENABLE);

    nvic_init_def.NVIC_IRQChannel = timer_info->irqn;
    nvic_init_def.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init_def.NVIC_IRQChannelSubPriority = 1;
    nvic_init_def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_def);

    TIM_Cmd(timer, ENABLE);
}

void pwm_init(TIM_TypeDef *timer, int channel, uint32_t frequency, uint16_t period_count)
{
    const struct timer_info *timer_info = timer_info_find(timer);
    TIM_OCInitTypeDef timer_oc_init_def;

    TRACE("timer %p, channel %d, frequency %u, period_count %u.\n", timer, channel, frequency, period_count);

    if (!timer_info || channel < 1 || channel > 4 || !frequency || !period_count)
    {
        TRACE("Invalid argument.\n");
        return;
    }

    timer_time_base_init(timer_info, true, SYSTEM_CLOCK_HZ / frequency / period_count - 1, period_count - 1);

    TIM_OCStructInit(&timer_oc_init_def);
    timer_oc_init_def.TIM_OCMode = TIM_OCMode_PWM1;
    timer_oc_init_def.TIM_OutputState = TIM_OutputState_Enable;
    timer_oc_init_def.TIM_Pulse = 0;
    timer_oc_init_def.TIM_OCPolarity = TIM_OCPolarity_High;

    switch (channel)
    {
        case 1:
            gpio_init(timer_info->channel1_gpio, timer_info->channel1_pin, GPIO_Mode_AF_PP);
            TIM_OC1Init(timer, &timer_oc_init_def);
            break;
        case 2:
            gpio_init(timer_info->channel2_gpio, timer_info->channel2_pin, GPIO_Mode_AF_PP);
            TIM_OC2Init(timer, &timer_oc_init_def);
            break;
        case 3:
            gpio_init(timer_info->channel3_gpio, timer_info->channel3_pin, GPIO_Mode_AF_PP);
            TIM_OC3Init(timer, &timer_oc_init_def);
            break;
        case 4:
            gpio_init(timer_info->channel4_gpio, timer_info->channel4_pin, GPIO_Mode_AF_PP);
            TIM_OC4Init(timer, &timer_oc_init_def);
            break;
        default:
            assert(0); /* Should never be here. */
    }

    TRACE("Initialized %s, channel %d.\n", timer_info->name, channel);

    TIM_Cmd(timer, ENABLE);
}

void servo_init(TIM_TypeDef *timer, int channel)
{
    TRACE("timer %p, channel %d.\n", timer, channel);
    pwm_init(timer, channel, SERVO_FREQUENCY, SERVO_PERIOD_COUNT);
}

void servo_set_angle(TIM_TypeDef *timer, int channel, uint32_t angle)
{
    TRACE("timer %p, channel %d, angle %u.\n", timer, channel, angle);

    /* Angle should be 0~180. */
    if (angle > 180)
    {
        TRACE("Invalid angle %u.\n", angle);
        return;
    }

    pwm_set_pulse(timer, channel, SERVO_PERIOD_COUNT / 40 + SERVO_PERIOD_COUNT * angle / 1800 );
}

void pwm_set_pulse(TIM_TypeDef *timer, int channel, uint16_t pulse)
{
    TRACE("timer %p, channel %d, pulse %u.\n", timer, channel, pulse);

    if (!timer || channel < 1 || channel >= 5)
    {
        TRACE("Invalid argument.\n", timer);
        return;
    }

    switch (channel)
    {
        case 1:
            TIM_SetCompare1(timer, pulse);
            break;
        case 2:
            TIM_SetCompare2(timer, pulse);
            break;
        case 3:
            TIM_SetCompare3(timer, pulse);
            break;
        case 4:
            TIM_SetCompare4(timer, pulse);
            break;
        default:
            assert(0); /* Should never be here. */
    }
}

void delay_us(uint32_t us)
{
    SysTick->LOAD = 72 * us;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    while (!(SysTick->CTRL & 0x00010000)) {}
    SysTick->CTRL = 4;
}

void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}

bool debug_init(USART_TypeDef *usart, GPIO_TypeDef *debug_led_gpio, uint16_t debug_led_pin)
{
    /* We need to disable IO buffering before any printf call.
     * _sbrk returns NULL, making IO buffer for printf fail to allocate.
     * if we don't disable buffering, printf will get stuck. */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    if (!usart_init(usart))
        return false;

    if (debug_led_gpio && debug_led_pin)
    {
        if (!gpio_init(debug_led_gpio, debug_led_pin, GPIO_Mode_Out_PP))
            return false;
    }

    debug_usart = usart;
    printf("\n\n\n=========================== %s started ===========================\n\n", __PROJECT_NAME__);
    TRACE("Debug channel inited on %s.\n", usart_name(usart));

    return true;
}

void debug_trace(const char *file, int line, const char *func, const char *format, ...)
{
    va_list args;
    printf("%s: %s(%d): %s: ", __PROJECT_NAME__, file, line, func);
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

int __io_putchar(int ch)
{
    if (!debug_usart)
        return 0;
    usart_send_byte(debug_usart, ch);
    return ch;
}

void __assert_func(const char *file, int line, const char *func, const char *expr)
{
    debug_trace(strrchr(file, '/') + 1, line, func, "ASSERT FAILED: (%s).\n", expr);

    while (1)
    {
        if (debug_led_gpio && debug_led_pin)
        {
            GPIO_WriteBit(debug_led_gpio, debug_led_pin, Bit_SET);
            delay_ms(500);
            GPIO_WriteBit(debug_led_gpio, debug_led_pin, Bit_RESET);
            delay_ms(500);
        }
    }
}

void assert_failed(uint8_t *file, uint32_t line)
{
    __assert_func((const char *)file, line, "assert_param", "Param assert failed");
}
