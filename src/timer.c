/*
 * STM32 timer helper functions.
 */

#include "stm32.h"

#define SYSTEM_CLOCK_HZ    (72 * 1000 * 1000)
#define SERVO_FREQUENCY    (50)
#define SERVO_PERIOD_COUNT (2000)

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

void timer_pwm_init(TIM_TypeDef *timer, int channel, uint32_t frequency, uint16_t period_count)
{
    const struct timer_info *timer_info = timer_info_find(timer);
    TIM_OCInitTypeDef timer_oc_init_def;

    if (!timer_info)
    {
        TRACE("Invalid timer %p.\n", timer);
        return;
    }

    if (channel < 1 || channel > 4 || !frequency || !period_count)
    {
        TRACE("Invalid argument: channel %d, frequency %u, period_count %u.\n",
                channel, frequency, period_count);
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

void timer_servo_init(TIM_TypeDef *timer, int channel)
{
    TRACE("timer %p, channel %d.\n", timer, channel);
    timer_pwm_init(timer, channel, SERVO_FREQUENCY, SERVO_PERIOD_COUNT);
}

void timer_servo_set_angle(TIM_TypeDef *timer, int channel, uint32_t angle)
{
    TRACE("timer %p, channel %d, angle %u.\n", timer, channel, angle);

    /* Angle should be 0~180. */
    if (angle > 180)
    {
        TRACE("Invalid angle %u.\n", angle);
        return;
    }

    timer_pwm_set_pulse(timer, channel, SERVO_PERIOD_COUNT / 40 + SERVO_PERIOD_COUNT * angle / 1800 );
}

void timer_pwm_set_pulse(TIM_TypeDef *timer, int channel, uint16_t pulse)
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
