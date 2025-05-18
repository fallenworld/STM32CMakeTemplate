/*
 * STM32 USART helper functions.
 */

#include "stm32.h"

struct usart_info
{
    USART_TypeDef *usart;
    const char *name;
    uint32_t periph;
    struct gpio_pin tx_pin, rx_pin;
};

static const struct usart_info usart_info_list[] =
{
    {USART1, "USART1", RCC_APB2Periph_USART1, {GPIOA, GPIO_Pin_9}, {GPIOA, GPIO_Pin_10}},
    {USART2, "USART2", RCC_APB1Periph_USART2, {GPIOA, GPIO_Pin_2}, {GPIOA, GPIO_Pin_3}},
};

const struct usart_info *usart_info_find(const USART_TypeDef *usart)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(usart_info_list); ++i)
    {
        if (usart_info_list[i].usart == usart)
            return &usart_info_list[i];
    }
    return NULL;
}

const char *usart_name(const USART_TypeDef *usart)
{
    const struct usart_info *usart_info = usart_info_find(usart);
    if (!usart_info)
        return "(Invalid USART)";
    return usart_info->name;
}

bool usart_init(USART_TypeDef *usart)
{
    const struct usart_info *usart_info = usart_info_find(usart);
    USART_InitTypeDef usart_init_def;

    if (!usart_info)
    {
        TRACE("Invalid usart %p.\n", usart);
        return false;
    }

    if (!gpio_pin_init(&usart_info->tx_pin, GPIO_Mode_AF_PP)
            || !gpio_pin_init(&usart_info->rx_pin,  GPIO_Mode_IPU))
    {
        TRACE("Failed to init TX/RX pins.\n");
        return false;
    }

    if (usart == USART1)
        abp2_periph_enable(usart_info->periph);
    else
        abp1_periph_enable(usart_info->periph);

    USART_StructInit(&usart_init_def);
    USART_Init(usart, &usart_init_def);
    USART_Cmd(usart, ENABLE);

    TRACE("Inited %s.\n", usart_info->name);

    return true;
}

void usart_send_byte(USART_TypeDef *usart, uint8_t byte)
{
    USART_SendData(usart, byte);
    while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET) {}
}

void usart_send_str(USART_TypeDef *usart, const char *str)
{
    while (*str)
    {
        usart_send_byte(usart, *str);
        ++str;
    }
}

bool usart_has_data(USART_TypeDef *usart)
{
    return USART_GetFlagStatus(usart, USART_FLAG_RXNE) == SET;
}

uint8_t usart_wait_byte(USART_TypeDef *usart)
{
    while (!usart_has_data(usart)) {}
    return USART_ReceiveData(usart);
}
