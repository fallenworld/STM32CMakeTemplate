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

    servo_init(TIM2, 1);
    servo_set_angle(TIM2, 1, 180);

    while (1)
    {

    }
}
