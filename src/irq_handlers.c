/*
 * Interrupt request handlers.
 */

#include "stm32.h"

irq_handler exti_handlers[15];
irq_handler usart_handlers[2];

static uint8_t exti_handler_index(uint32_t exti_line)
{
    return count_trailing_zeros(exti_line);
}

static uint8_t usart_handler_index(USART_TypeDef *usart)
{
    if (usart == USART1)
        return 0;
    if (usart == USART2)
        return 1;
    return UINT8_MAX;
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

static void usart_common_handler(USART_TypeDef *usart)
{
    if (USART_GetITStatus(usart, USART_IT_RXNE) == SET)
    {
        uint8_t index = usart_handler_index(usart);
        irq_handler handler = usart_handlers[index];

        if (handler)
            handler();
        else
            TRACE("USART%u handler not set.\n", index + 1);

        USART_ClearITPendingBit(usart, USART_IT_RXNE);
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

bool usart_set_handler(USART_TypeDef *usart, irq_handler handler)
{
    uint32_t index = usart_handler_index(usart);

    if (index >= ARRAY_SIZE(usart_handlers))
    {
        TRACE("Invalid usart %p.\n", usart);
        return false;
    }

    usart_handlers[index] = handler;
    TRACE("Set USART%d handler to %p.\n", index + 1, handler);

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

void USART1_IRQHandler(void)
{
    usart_common_handler(USART1);
}

void USART2_IRQHandler(void)
{
    usart_common_handler(USART2);
}
