/*
 * STM32 timer helper functions.
 */

#include "stm32.h"

#define SYSTEM_CLOCK_HZ (72 * 1000 * 1000)
#define SERVO_FREQUENCY 50
#define SERVO_PERIOD_COUNT 2000
#define INPUT_CAPTURE_PERIOD_COUNT (UINT16_MAX + 1)
#define INPUT_CAPTURE_PRESCALER_FACTOR 72

#define TIM_CHANNEL_NUM(channel) ((channel) / (TIM_Channel_2 - TIM_Channel_1))

#define CHECK_TIMER(timer, timer_info, fail_ret) do \
{                                                   \
    if (!(timer_info))                              \
    {                                               \
        TRACE("Invalid timer %p.\n", timer);        \
        return fail_ret;                            \
    }                                               \
} while (0)

#define CHECK_TIMER_CHANNEL(timer, timer_info, channel, fail_ret) do \
{                                                                    \
    CHECK_TIMER(timer, timer_info, fail_ret);                        \
    if (!IS_TIM_CHANNEL(channel))                                    \
    {                                                                \
        TRACE("Invalid channel: channel %u.\n", channel);            \
        return fail_ret;                                             \
    }                                                                \
} while (0)

struct timer_info
{
    TIM_TypeDef *timer;
    const char *name;
    uint32_t periph;
    enum IRQn irqn;
    struct gpio_pin channel_pins[4];
};

static const struct timer_info timer_info_list[] =
{
    {
        TIM2, "TIM2", RCC_APB1Periph_TIM2, TIM2_IRQn,
        {
            {GPIOA, GPIO_Pin_0},
            {GPIOA, GPIO_Pin_1},
            {GPIOA, GPIO_Pin_2},
            {GPIOA, GPIO_Pin_3},
        }
    },
    {
        TIM3, "TIM3", RCC_APB1Periph_TIM3, TIM3_IRQn,
        {
            {GPIOA, GPIO_Pin_6},
            {GPIOA, GPIO_Pin_7},
            {GPIOB, GPIO_Pin_0},
            {GPIOB, GPIO_Pin_1},
        }
    },
    {
        TIM4, "TIM4", RCC_APB1Periph_TIM4, TIM4_IRQn,
        {
            {GPIOB, GPIO_Pin_6},
            {GPIOB, GPIO_Pin_7},
            {GPIOB, GPIO_Pin_8},
            {GPIOB, GPIO_Pin_9},
        }
    },
};

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

static void timer_time_base_init(const struct timer_info *timer_info,
        bool internal_clock, uint32_t prescaler_factor, uint32_t period_count)
{
    TIM_TimeBaseInitTypeDef timer_init_def;

    abp1_periph_enable(timer_info->periph);

    if (internal_clock)
        TIM_InternalClockConfig(timer_info->timer);
    else
        TIM_ETRClockMode2Config(timer_info->timer, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x0f);

    TIM_TimeBaseStructInit(&timer_init_def);
    timer_init_def.TIM_Prescaler = prescaler_factor - 1;
    timer_init_def.TIM_Period = period_count - 1;
    TIM_TimeBaseInit(timer_info->timer, &timer_init_def);

    TRACE("Inited %s time base, use %s clock, prescaler_factor %u, period_count %u.\n",
            timer_info->name, internal_clock ? "internal" : "external", prescaler_factor, period_count);
}

bool timer_update_init(TIM_TypeDef *timer, bool internal_clock, uint32_t prescaler_factor, uint32_t period_count)
{
    const struct timer_info *timer_info = timer_info_find(timer);
    NVIC_InitTypeDef nvic_init_def;

    CHECK_TIMER(timer, timer_info, false);

    timer_time_base_init(timer_info, internal_clock, prescaler_factor, period_count);

    TIM_ClearFlag(timer, TIM_FLAG_Update);
    TIM_ITConfig(timer, TIM_IT_Update, ENABLE);

    nvic_init_def.NVIC_IRQChannel = timer_info->irqn;
    nvic_init_def.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init_def.NVIC_IRQChannelSubPriority = 1;
    nvic_init_def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_def);

    TIM_Cmd(timer, ENABLE);
    return true;
}

bool timer_pwm_init(TIM_TypeDef *timer, uint16_t channel, uint32_t frequency, uint32_t period_count)
{
    uint32_t prescaler_factor = SYSTEM_CLOCK_HZ / (frequency * period_count);
    const struct timer_info *timer_info = timer_info_find(timer);
    TIM_OCInitTypeDef timer_oc_init_def;

    CHECK_TIMER_CHANNEL(timer, timer_info, channel, false);

    if (!frequency || !period_count)
    {
        TRACE("Invalid frequency or period count: frequency %u, period_count %u.\n", frequency, period_count);
        return false;
    }

    if (SYSTEM_CLOCK_HZ % (frequency * period_count) != 0)
    {
        TRACE("Clock frequency %u can't be evenly divided by %lu * %lu, actual output frequency: %lu.\n",
                SYSTEM_CLOCK_HZ, frequency, period_count, SYSTEM_CLOCK_HZ / (prescaler_factor * period_count));
    }

    if (!gpio_pin_init(&timer_info->channel_pins[TIM_CHANNEL_NUM(channel)], GPIO_Mode_AF_PP))
        return false;

    timer_time_base_init(timer_info, true, prescaler_factor, period_count);

    TIM_OCStructInit(&timer_oc_init_def);
    timer_oc_init_def.TIM_OCMode = TIM_OCMode_PWM1;
    timer_oc_init_def.TIM_OutputState = TIM_OutputState_Enable;
    timer_oc_init_def.TIM_Pulse = 0;
    timer_oc_init_def.TIM_OCPolarity = TIM_OCPolarity_High;

    switch (channel)
    {
        case TIM_Channel_1:
            TIM_OC1Init(timer, &timer_oc_init_def);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(timer, &timer_oc_init_def);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(timer, &timer_oc_init_def);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(timer, &timer_oc_init_def);
            break;
        default:
            assert(0); /* Should never be here. */
    }

    TIM_Cmd(timer, ENABLE);
    TRACE("Inited %s, channel %d.\n", timer_info->name, channel);

    return true;
}

bool timer_pwm_set_pulse(TIM_TypeDef *timer, uint16_t channel, uint16_t pulse)
{
    const struct timer_info *timer_info = timer_info_find(timer);

    CHECK_TIMER_CHANNEL(timer, timer_info, channel, false);

    switch (channel)
    {
        case TIM_Channel_1:
            TIM_SetCompare1(timer, pulse);
            break;
        case TIM_Channel_2:
            TIM_SetCompare2(timer, pulse);
            break;
        case TIM_Channel_3:
            TIM_SetCompare3(timer, pulse);
            break;
        case TIM_Channel_4:
            TIM_SetCompare4(timer, pulse);
            break;
        default:
            assert(0); /* Should never be here. */
    }

    TRACE("Set %s channel %d pulse to %u.\n", timer_info->name, channel, pulse);

    return true;
}

bool timer_input_capture_init(TIM_TypeDef *timer, uint16_t channel, bool use_pwm_input)
{
    const struct timer_info *timer_info = timer_info_find(timer);
    TIM_ICInitTypeDef timer_ic_init_def;

    CHECK_TIMER_CHANNEL(timer, timer_info, channel, 0);

    if (use_pwm_input && channel != TIM_Channel_1 && channel != TIM_Channel_2)
    {
        TRACE("Invalid channel %u for PWM input, use TIM1/TIM2.\n", TIM_CHANNEL_NUM(channel));
        return false;
    }

    if (!gpio_pin_init(&timer_info->channel_pins[TIM_CHANNEL_NUM(channel)], GPIO_Mode_IN_FLOATING))
        return false;

    timer_time_base_init(timer_info, true, INPUT_CAPTURE_PRESCALER_FACTOR, INPUT_CAPTURE_PERIOD_COUNT);

    TIM_ICStructInit(&timer_ic_init_def);
    timer_ic_init_def.TIM_Channel = channel;
    timer_ic_init_def.TIM_ICFilter = 0x0f;
    if (use_pwm_input)
        TIM_PWMIConfig(timer, &timer_ic_init_def);
    else
        TIM_ICInit(timer, &timer_ic_init_def);

    TIM_SelectInputTrigger(timer, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(timer, TIM_SlaveMode_Reset);

    TIM_Cmd(timer, ENABLE);
    TRACE("Inited input capture on %s, channel %d.\n", timer_info->name, channel);

    return true;
}

uint32_t timer_input_capture_get_frequency(TIM_TypeDef *timer, uint16_t channel)
{
    const struct timer_info *timer_info = timer_info_find(timer);

    CHECK_TIMER_CHANNEL(timer, timer_info, channel, false);

    switch (channel)
    {
        case TIM_Channel_1:
            return SYSTEM_CLOCK_HZ / INPUT_CAPTURE_PRESCALER_FACTOR / (TIM_GetCapture1(timer) + 1);
        case TIM_Channel_2:
            return SYSTEM_CLOCK_HZ / INPUT_CAPTURE_PRESCALER_FACTOR / (TIM_GetCapture2(timer) + 1);
        case TIM_Channel_3:
            return SYSTEM_CLOCK_HZ / INPUT_CAPTURE_PRESCALER_FACTOR / (TIM_GetCapture3(timer) + 1);
        case TIM_Channel_4:
            return SYSTEM_CLOCK_HZ / INPUT_CAPTURE_PRESCALER_FACTOR / (TIM_GetCapture4(timer) + 1);
        default:
            assert(0); /* Should never be here. */
    }
}

uint16_t timer_input_capture_get_duty(TIM_TypeDef *timer, uint16_t channel)
{
    const struct timer_info *timer_info = timer_info_find(timer);

    CHECK_TIMER_CHANNEL(timer, timer_info, channel, false);

    if (channel == TIM_Channel_1)
        return (TIM_GetCapture2(timer) + 1) * 100 / (TIM_GetCapture1(timer) + 1);
    if (channel == TIM_Channel_2)
        return (TIM_GetCapture1(timer) + 1) * 100 / (TIM_GetCapture2(timer) + 1);

    TRACE("Invalid channel %u for PWM input, use TIM1/TIM2.\n", TIM_CHANNEL_NUM(channel));

    return 0;
}

bool servo_init(TIM_TypeDef *timer, uint16_t channel)
{
    TRACE("timer %p, channel %d.\n", timer, channel);
    return timer_pwm_init(timer, channel, SERVO_FREQUENCY, SERVO_PERIOD_COUNT);
}

bool servo_set_angle(TIM_TypeDef *timer, uint16_t channel, uint32_t angle)
{
    TRACE("timer %p, channel %d, angle %u.\n", timer, channel, angle);

    /* Angle should be 0~180. */
    if (angle > 180)
    {
        TRACE("Invalid angle %u.\n", angle);
        return false;
    }

    return timer_pwm_set_pulse(timer, channel, SERVO_PERIOD_COUNT / 40 + SERVO_PERIOD_COUNT * angle / 1800);
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
