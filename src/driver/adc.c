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
    DMA_Channel_TypeDef *dma_channel;
    struct gpio_pin channel_pins[10];
};

static const struct adc_info adc_info[] =
{
    {
        ADC1, "ADC1", RCC_APB2Periph_ADC1, DMA1_Channel1,
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
        ADC2, "ADC2", RCC_APB2Periph_ADC2, NULL,
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

static void adc_calibrate(ADC_TypeDef *adc)
{
    ADC_ResetCalibration(adc);
    while (ADC_GetResetCalibrationStatus(adc) != RESET) {}
    ADC_StartCalibration(adc);
    while (ADC_GetCalibrationStatus(adc) != RESET) {}
}

static void adc_init(const struct adc_info *adc_info, uint8_t channel_count, bool scan_mode, bool continuous_mode)
{
    ADC_InitTypeDef adc_init_type_def;

    abp2_periph_enable(adc_info->periph);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    ADC_StructInit(&adc_init_type_def);
    adc_init_type_def.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc_init_type_def.ADC_NbrOfChannel = channel_count;
    adc_init_type_def.ADC_ScanConvMode = scan_mode ? ENABLE : DISABLE;
    adc_init_type_def.ADC_ContinuousConvMode = continuous_mode ? ENABLE : DISABLE;
    ADC_Init(adc_info->adc, &adc_init_type_def);
}

bool adc_single_init(ADC_TypeDef *adc)
{
    const struct adc_info *adc_info = adc_info_find(adc);

    CHECK_ADC(adc, adc_info, false);

    adc_init(adc_info, 1, false, false);
    ADC_Cmd(adc, ENABLE);
    adc_calibrate(adc);

    TRACE("Inited %s.\n", adc_info->name);

    return true;
}

bool adc_dma_init(ADC_TypeDef *adc, uint8_t adc_channel_count, void *dst_addr)
{
    const struct adc_info *adc_info = adc_info_find(adc);

    CHECK_ADC(adc, adc_info, false);

    if (!adc_info->dma_channel)
    {
        TRACE("%s does not support DMA.\n", adc_info->name);
        return false;
    }

    adc_init(adc_info, adc_channel_count, true, true);
    dma_init(adc_info->dma_channel, (void *)&adc->DR, dst_addr, sizeof(uint16_t), false, false, true);
    dma_start_transfer(adc_info->dma_channel, adc_channel_count);
    ADC_DMACmd(adc, ENABLE);
    ADC_Cmd(adc, ENABLE);
    adc_calibrate(adc);

    TRACE("Inited %s.\n", adc_info->name);

    return true;
}

bool adc_init_channel(ADC_TypeDef *adc, uint8_t channel)
{
    const struct adc_info *adc_info = adc_info_find(adc);
    CHECK_ADC_CHANNEL(adc, adc_info, channel, false);
    return gpio_pin_init(&adc_info->channel_pins[channel], GPIO_Mode_AIN);
}

void adc_start_single_convert(ADC_TypeDef *adc, uint8_t channel)
{
    ADC_RegularChannelConfig(adc, channel, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(adc, ENABLE);
}

uint16_t adc_wait_value(ADC_TypeDef *adc)
{
    while (ADC_GetFlagStatus(adc, ADC_FLAG_EOC) != SET) {}
    return ADC_GetConversionValue(adc);
}
