/*
 * STM32 main entry point.
 */

#include "stm32.h"

#define DEBUG_LED_GPIO GPIOA
#define DEBUG_LED_PIN  GPIO_Pin_13

void func(void)
{
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0)
        TRACE("Trigger falling edge.\n");
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    debug_init(USART1, DEBUG_LED_GPIO, DEBUG_LED_PIN);
    exti_init(GPIOA, GPIO_Pin_1, func, EXTI_Trigger_Falling, 1, 1);

    while (1)
    {

    }
}
