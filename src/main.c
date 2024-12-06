/*
 * STM32 main entry point.
 */

#include "stm32f10x.h"

static void delay_us(uint32_t us)
{
    SysTick->LOAD = 72 * us;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    while (!(SysTick->CTRL & 0x00010000)) {}
    SysTick->CTRL = 4;
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

int main(void)
{
    while (1)
    {
        delay_ms(100);
    }
}

