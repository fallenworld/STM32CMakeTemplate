/*
 * USART functions.
 */

#include "stm32.h"

void usart_init(USART_TypeDef *usart_x)
{
    USART_InitTypeDef usart_init_def;
    uint16_t tx_pin, rx_pin;
    GPIO_TypeDef *gpio_x;

    if (usart_x == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        gpio_x = GPIOA;
        tx_pin = GPIO_Pin_9;
        rx_pin = GPIO_Pin_10;
    }
    else if (usart_x == USART2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        gpio_x = GPIOA;
        tx_pin = GPIO_Pin_2;
        rx_pin = GPIO_Pin_3;
    }
    else
    {
        return;
    }

    gpio_init(gpio_x, tx_pin, GPIO_Mode_AF_PP);
    gpio_init(gpio_x, rx_pin, GPIO_Mode_IN_FLOATING);

    usart_init_def.USART_BaudRate = 9600;
    usart_init_def.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_def.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_def.USART_Parity = USART_Parity_No;
    usart_init_def.USART_StopBits = USART_StopBits_1;
    usart_init_def.USART_WordLength = USART_WordLength_8b;
    USART_Init(usart_x, &usart_init_def);
    USART_Cmd(usart_x, ENABLE);
}

void usart_send_byte(USART_TypeDef *usart_x, uint8_t byte)
{
    USART_SendData(usart_x, byte);
    while (USART_GetFlagStatus(usart_x, USART_FLAG_TXE) == RESET) {}
}

void usart_send_str(USART_TypeDef *usart_x, const char *str)
{
    while (*str)
    {
        usart_send_byte(usart_x, *str);
        ++str;
    }
}

bool usart_has_data(USART_TypeDef *usart_x)
{
    return USART_GetFlagStatus(usart_x, USART_FLAG_RXNE) == SET;
}

uint8_t usart_wait_byte(USART_TypeDef *usart_x)
{
    while (!usart_has_data(usart_x)) {}
    return USART_ReceiveData(usart_x);
}
