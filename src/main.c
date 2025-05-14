/*
 * STM32 main entry point.
 */

#include "stm32.h"


int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    DEBUG_INIT(USART1);

    while (1)
    {

    }
}
