/*
 * STM32 main entry point.
 */

#include "stm32.h"

#define DEBUG_LED_GPIO GPIOA
#define DEBUG_LED_PIN  GPIO_Pin_13

void func(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0)
        TRACE("Trigger falling edge on PB9.\n");
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    debug_init(USART1, DEBUG_LED_GPIO, DEBUG_LED_PIN);
    exti_init(GPIOB, GPIO_Pin_9, func, EXTI_Trigger_Falling, 1, 1);

    timer_pwm_init(TIM2, TIM_Channel_1, 1000, 100);
    timer_pwm_set_pulse(TIM2, TIM_Channel_1, 16);
    timer_input_capture_init(TIM3, TIM_Channel_1, true);

    while (1)
    {
        TRACE("Frequency: %u, duty %u.\n",
                timer_input_capture_get_frequency(TIM3, TIM_Channel_1),
                timer_input_capture_get_duty(TIM3, TIM_Channel_1));
        delay_ms(500);
    }
}
