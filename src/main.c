/*
 * STM32 main entry point.
 */

#include "stm32.h"

#define DEBUG_LED_GPIO GPIOA
#define DEBUG_LED_PIN  GPIO_Pin_13

#define TEST_EXTI (1 << 0)
#define TEST_PWMI (1 << 1)
#define TEST_ENCODER (1 << 2)
#define TEST_ADC (1 << 3)

void exti_irq_handler(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0)
        TRACE("Trigger falling edge on PB9.\n");
}

int main(void)
{
    uint32_t test_case = TEST_EXTI | TEST_ADC;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    debug_init(USART1, DEBUG_LED_GPIO, DEBUG_LED_PIN);

    if (test_case & TEST_EXTI)
        exti_init(GPIOB, GPIO_Pin_9, exti_irq_handler, EXTI_Trigger_Falling, 1, 1);

    if (test_case & TEST_PWMI)
    {
        timer_pwm_init(TIM2, TIM_Channel_1, 1000, 100);
        timer_pwm_set_pulse(TIM2, TIM_Channel_1, 16);
        timer_input_capture_init(TIM3, TIM_Channel_1, true);
    }
    if (test_case & TEST_ENCODER)
    {
        encoder_init(TIM3);
    }
    if (test_case & TEST_ADC)
    {
        adc_init(ADC1);
        adc_init_channel(ADC1, ADC_Channel_2);
    }

    while (1)
    {
        if (test_case & TEST_PWMI)
        {
            TRACE("Frequency: %u, duty %u.\n",
                    timer_input_capture_get_frequency(TIM3, TIM_Channel_1),
                    timer_input_capture_get_duty(TIM3, TIM_Channel_1));
            delay_ms(20);
        }
        if (test_case & TEST_ENCODER)
        {
            printf("encoder count: %d.\n", (int16_t)encoder_get_count(TIM3));
            delay_ms(20);
        }
        if (test_case & TEST_ADC)
        {
            adc_start_convert(ADC1, ADC_Channel_2);
            printf("adc value: %u.\n", adc_wait_value(ADC1));
            delay_ms(20);
        }
    }
}
