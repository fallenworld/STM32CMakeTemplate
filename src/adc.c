/*
 * Analog-to-digital converter functions.
 */

#include "stm32.h"

#define CHECK_ADC(adc, adc_info, fail_ret) do \
{                                             \
    if (!(adc_info))                          \
    {                                         \
        TRACE("Invalid adc %p.\n", adc);      \
        return fail_ret;                      \
    }                                         \
} while (0)

#define CHECK_ADC_CHANNEL(adc, adc_info, channel, fail_ret) do \
{                                                              \
    CHECK_ADC(adc, adc_info, fail_ret);                        \
    if (!IS_ADC_CHANNEL(channel))                              \
    {                                                          \
        TRACE("Invalid channel: channel %#x.\n", channel);     \
        return fail_ret;                                       \
    }                                                          \
} while (0)


struct adc_info
{
    ADC_TypeDef *adc;
    const char *name;
    uint32_t periph;
    struct gpio_pin channel_pins[10];
};

static const struct adc_info adc_info[] =
{
    {
        ADC1, "ADC1", RCC_APB2Periph_ADC1,
        {
            {GPIOA, GPIO_Pin_0},
            {GPIOA, GPIO_Pin_1},
            {GPIOA, GPIO_Pin_2},
            {GPIOA, GPIO_Pin_3},
            {GPIOA, GPIO_Pin_4},
            {GPIOA, GPIO_Pin_5},
            {GPIOA, GPIO_Pin_6},
            {GPIOA, GPIO_Pin_7},
            {GPIOB, GPIO_Pin_0},
            {GPIOB, GPIO_Pin_1},
        }
    },
    {
        ADC2, "ADC2", RCC_APB2Periph_ADC2,
        {
            {GPIOA, GPIO_Pin_0},
            {GPIOA, GPIO_Pin_1},
            {GPIOA, GPIO_Pin_2},
            {GPIOA, GPIO_Pin_3},
            {GPIOA, GPIO_Pin_4},
            {GPIOA, GPIO_Pin_5},
            {GPIOA, GPIO_Pin_6},
            {GPIOA, GPIO_Pin_7},
            {GPIOB, GPIO_Pin_0},
            {GPIOB, GPIO_Pin_1},
        }
    },
};

static const struct adc_info *adc_info_find(const ADC_TypeDef *adc)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(adc_info); ++i)
    {
        if (adc_info[i].adc == adc)
            return &adc_info[i];
    }
    return NULL;
}

bool adc_init(ADC_TypeDef *adc)
{
    const struct adc_info *adc_info = adc_info_find(adc);
    ADC_InitTypeDef adc_init_type_def;

    CHECK_ADC(adc, adc_info, false);

    abp2_periph_enable(adc_info->periph);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_StructInit(&adc_init_type_def);
    adc_init_type_def.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_Init(adc, &adc_init_type_def);

    ADC_Cmd(adc, ENABLE);

    ADC_ResetCalibration(adc);
    while (ADC_GetResetCalibrationStatus(adc) != RESET) {}
    ADC_StartCalibration(adc);
    while (ADC_GetCalibrationStatus(adc) != RESET) {}

    TRACE("Inited %s.\n", adc_info->name);

    return true;
}

bool adc_init_channel(ADC_TypeDef *adc, uint8_t channel)
{
    const struct adc_info *adc_info = adc_info_find(adc);
    CHECK_ADC_CHANNEL(adc, adc_info, channel, false);
    return gpio_pin_init(&adc_info->channel_pins[channel], GPIO_Mode_AIN);
}

void adc_start_convert(ADC_TypeDef *adc, uint8_t channel)
{
    ADC_RegularChannelConfig(adc, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(adc, ENABLE);
}

uint16_t adc_wait_value(ADC_TypeDef *adc)
{
    while (ADC_GetFlagStatus(adc, ADC_FLAG_EOC) != SET) {}
    return ADC_GetConversionValue(adc);
}
