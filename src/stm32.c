/*
 * STM32 helper functions.
 */

#include "stm32.h"

static USART_TypeDef *debug_usart = NULL;
static char print_buffer[128];

struct gpio_info
{
    GPIO_TypeDef *gpio;
    const char *name;
    uint32_t periph;
    uint8_t port_source;
};

static const struct gpio_info gpio_info_list[] =
{
    {GPIOA, "GPIOA", RCC_APB2Periph_GPIOA, GPIO_PortSourceGPIOA},
    {GPIOB, "GPIOB", RCC_APB2Periph_GPIOB, GPIO_PortSourceGPIOB},
    {GPIOC, "GPIOC", RCC_APB2Periph_GPIOC, GPIO_PortSourceGPIOC},
    {GPIOD, "GPIOD", RCC_APB2Periph_GPIOD, GPIO_PortSourceGPIOD},
    {GPIOE, "GPIOE", RCC_APB2Periph_GPIOE, GPIO_PortSourceGPIOE},
    {GPIOF, "GPIOF", RCC_APB2Periph_GPIOF, GPIO_PortSourceGPIOF},
    {GPIOG, "GPIOG", RCC_APB2Periph_GPIOG, GPIO_PortSourceGPIOG},
};

static const struct gpio_info *gpio_info_find(const GPIO_TypeDef *gpio)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(gpio_info_list); ++i)
    {
        if (gpio_info_list[i].gpio == gpio)
            return &gpio_info_list[i];
    }
    return NULL;
}

bool is_abp1_periph_enabled(uint32_t periph)
{
    return !!(RCC->APB1ENR & periph);
}

bool is_abp2_periph_enabled(uint32_t periph)
{
    return !!(RCC->APB2ENR & periph);
}

void abp1_periph_enable(uint32_t periph)
{
    if (!is_abp1_periph_enabled(periph))
        RCC_APB1PeriphClockCmd(periph, ENABLE);
}

void abp2_periph_enable(uint32_t periph)
{
    if (!is_abp2_periph_enabled(periph))
        RCC_APB2PeriphClockCmd(periph, ENABLE);
}

void gpio_init(GPIO_TypeDef *gpio, uint16_t pins, GPIOMode_TypeDef mode)
{
    const struct gpio_info *gpio_info = gpio_info_find(gpio);
    GPIO_InitTypeDef gpio_init_def;

    if (!gpio_info)
    {
        TRACE("Invalid gpio %p.\n", gpio);
        return;
    }

    abp2_periph_enable(gpio_info->periph);

    gpio_init_def.GPIO_Mode = mode;
    gpio_init_def.GPIO_Pin = pins;
    gpio_init_def.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio, &gpio_init_def);

    TRACE("%s initialized.\n", gpio_info->name);
}

void exti_init(uint8_t port_source, uint32_t exti_line, uint8_t pin_source, uint8_t pin_exti_irqn,
        EXTITrigger_TypeDef trigger, uint8_t preemption_pri, uint8_t sub_pri)
{
    EXTI_InitTypeDef exti_init_def;
    NVIC_InitTypeDef nvic_init_def;

    abp2_periph_enable(RCC_APB2Periph_AFIO);

    GPIO_EXTILineConfig(port_source, pin_source);

    exti_init_def.EXTI_Line = exti_line;
    exti_init_def.EXTI_Trigger = trigger;
    exti_init_def.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init_def.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init_def);

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    nvic_init_def.NVIC_IRQChannel = pin_exti_irqn;
    nvic_init_def.NVIC_IRQChannelPreemptionPriority = preemption_pri;
    nvic_init_def.NVIC_IRQChannelSubPriority = sub_pri;
    nvic_init_def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_def);
}

void usart_init(USART_TypeDef *usart, uint32_t usart_periph)
{
    USART_InitTypeDef usart_init_def;

    if (usart == USART1)
        abp2_periph_enable(usart_periph);
    else
        abp1_periph_enable(usart_periph);

    usart_init_def.USART_BaudRate = 9600;
    usart_init_def.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init_def.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart_init_def.USART_Parity = USART_Parity_No;
    usart_init_def.USART_StopBits = USART_StopBits_1;
    usart_init_def.USART_WordLength = USART_WordLength_8b;
    USART_Init(usart, &usart_init_def);

    USART_Cmd(usart, ENABLE);
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

void timer_init()
{
    TIM_TimeBaseInitTypeDef timer_init_def;
    NVIC_InitTypeDef nvic_init_def;

    abp1_periph_enable(RCC_APB1Periph_TIM2);

#if 0
    TIM_InternalClockConfig(TIM2);
#else
    TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x0f);
#endif
    timer_init_def.TIM_Prescaler = 1 - 1;
    timer_init_def.TIM_CounterMode = TIM_CounterMode_Up;
    timer_init_def.TIM_Period = 5 - 1;
    timer_init_def.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_init_def.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timer_init_def);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    nvic_init_def.NVIC_IRQChannel = TIM2_IRQn;
    nvic_init_def.NVIC_IRQChannelPreemptionPriority = 1;
    nvic_init_def.NVIC_IRQChannelSubPriority = 1;
    nvic_init_def.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init_def);

    TIM_Cmd(TIM2, ENABLE);
}

void delay_us(uint32_t us)
{
    SysTick->LOAD = 72 * us;
    SysTick->VAL = 0;
    SysTick->CTRL = 5;
    while (!(SysTick->CTRL & 0x00010000)) {}
    SysTick->CTRL = 4;
}

void delay_ms(uint32_t ms)
{
    while (ms--)
        delay_us(1000);
}

void debug_set_usart(USART_TypeDef *usart)
{
    debug_usart = usart;
}

void vprint(const char *format, va_list args)
{
    if (!debug_usart)
        return;
    vsnprintf(print_buffer, sizeof(print_buffer), format, args);
    usart_send_str(debug_usart, print_buffer);
}

void print(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprint(format, args);
    va_end(args);
}

void debug_trace(const char *file, int line, const char *func, const char *format, ...)
{
    va_list args;
    print("%s: %s: %d: %s(): ", __PROJECT_NAME__, file, line, func);
    va_start(args, format);
    vprint(format, args);
    va_end(args);
}
