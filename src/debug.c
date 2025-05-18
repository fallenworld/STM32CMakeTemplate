/*
 * STM32 debugging helper functions.
 */

#include "stm32.h"

static USART_TypeDef *debug_usart = NULL;
static GPIO_TypeDef *debug_led_gpio = NULL;
static uint16_t debug_led_pin = 0;
static char debug_buffer[256];
size_t debug_buffer_pos = 0;

bool debug_init(USART_TypeDef *usart, GPIO_TypeDef *debug_led_gpio, uint16_t debug_led_pin)
{
    /* We need to disable IO buffering before any printf call.
     * Our _sbrk returns NULL, making IO buffer for printf fail to allocate.
     * If we don't disable buffering, printf will get stuck. */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    if (!usart_init(usart))
        return false;

    if (debug_led_gpio && debug_led_pin)
    {
        if (!gpio_init(debug_led_gpio, debug_led_pin, GPIO_Mode_Out_PP))
            return false;
    }

    debug_usart = usart;
    printf("\n\n\n=========================== %s started ===========================\n\n", __PROJECT_NAME__);
    TRACE("Debug channel inited on %s.\n", usart_name(usart));

    return true;
}

void debug_trace(const char *file, int line, const char *func, const char *format, ...)
{
    va_list args;
    printf("%s: %s(%d): %s: ", __PROJECT_NAME__, file, line, func);
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

/* TODO: Use lock for multithread. */
const char *debug_buffer_vsprintf(const char *format, va_list args)
{
    char buffer[sizeof(debug_buffer) / 4];
    const char *ret;
    size_t size;

    size = vsnprintf(buffer, sizeof(buffer), format, args) + 1;
    if (size > sizeof(buffer))
    {
        strcpy(buffer + sizeof(buffer) - 4, "...");
        size = sizeof(buffer);
    }

    if (debug_buffer_pos + size > sizeof(debug_buffer))
        debug_buffer_pos = 0;
    size = sprintf(debug_buffer + debug_buffer_pos, "%s", buffer) + 1;

    ret = debug_buffer + debug_buffer_pos;
    debug_buffer_pos += size;

    return ret;
}

const char *debug_buffer_sprintf(const char *format, ...)
{
    const char *ret;
    va_list args;
    va_start(args, format);
    ret = debug_buffer_vsprintf(format, args);
    va_end( args );
    return ret;
}

int __io_putchar(int ch)
{
    if (!debug_usart)
        return 0;
    usart_send_byte(debug_usart, ch);
    return ch;
}

void __assert_func(const char *file, int line, const char *func, const char *expr)
{
    debug_trace(strrchr(file, '/') + 1, line, func, "ASSERT FAILED: (%s).\n", expr);

    while (1)
    {
        if (debug_led_gpio && debug_led_pin)
        {
            GPIO_WriteBit(debug_led_gpio, debug_led_pin, Bit_SET);
            delay_ms(500);
            GPIO_WriteBit(debug_led_gpio, debug_led_pin, Bit_RESET);
            delay_ms(500);
        }
    }
}

void assert_failed(uint8_t *file, uint32_t line)
{
    __assert_func((const char *)file, line, "assert_param", "Param assert failed");
}
