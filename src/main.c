/*
 * STM32 main entry point.
 */

#include "stm32.h"

#define DEBUG_LED_GPIO GPIOA
#define DEBUG_LED_PIN  GPIO_Pin_13

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    debug_init(USART1, DEBUG_LED_GPIO, DEBUG_LED_PIN);

    while (1)
    {

    }
}
