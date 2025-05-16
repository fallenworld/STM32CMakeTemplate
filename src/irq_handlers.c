/*
 * Interrupt request handlers.
 */

#include "stm32.h"

irq_handler exti_handlers[15];

static uint8_t exti_handler_index(uint32_t exti_line)
{
    return count_trailing_zeros(exti_line);
}

static void exti_common_handler(uint32_t exti_line)
{
    if (EXTI_GetITStatus(exti_line) == SET)
    {
        uint8_t index = exti_handler_index(exti_line);
        irq_handler func = exti_handlers[index];

        if (func)
            func();
        else
            TRACE("EXTI%u handler not set.\n", index);

        EXTI_ClearITPendingBit(exti_line);
    }
}

bool exti_set_handler(uint32_t exti_line, irq_handler handler)
{
    uint32_t index = exti_handler_index(exti_line);

    if (index >= ARRAY_SIZE(exti_handlers))
    {
        TRACE("Invalid exti_line %08x.\n", exti_line);
        return false;
    }

    exti_handlers[index] = handler;
    TRACE("Set EXTI%d handler to %p.\n", index, handler);

    return true;
}

void EXTI0_IRQHandler(void)
{
    exti_common_handler(EXTI_Line0);
}

void EXTI1_IRQHandler(void)
{
    exti_common_handler(EXTI_Line1);
}

void EXTI2_IRQHandler(void)
{
    exti_common_handler(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{
    exti_common_handler(EXTI_Line3);
}

void EXTI4_IRQHandler(void)
{
    exti_common_handler(EXTI_Line4);
}

void EXTI9_5_IRQHandler(void)
{
    exti_common_handler(EXTI_Line5);
    exti_common_handler(EXTI_Line6);
    exti_common_handler(EXTI_Line7);
    exti_common_handler(EXTI_Line8);
    exti_common_handler(EXTI_Line9);
}

void EXTI15_10_IRQHandler(void)
{
    exti_common_handler(EXTI_Line10);
    exti_common_handler(EXTI_Line11);
    exti_common_handler(EXTI_Line12);
    exti_common_handler(EXTI_Line13);
    exti_common_handler(EXTI_Line14);
    exti_common_handler(EXTI_Line15);
}
