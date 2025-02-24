/*
 * Debug functions.
 */

#include "stm32.h"

static USART_TypeDef *debug_usart;

void debug_init(USART_TypeDef *usart_x)
{
    debug_usart = usart_x;
}

void vprint(const char *format, va_list args)
{
    static char buffer[128];
    vsnprintf(buffer, sizeof(buffer), format, args);
    usart_send_str(debug_usart, buffer);
}

void print(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprint(format, args);
    va_end(args);
}

void debug_trace(const char *file, const char *func, int line, const char *format, ...)
{
    va_list args;
    print("%s:%d:%s(): ", file, line, func);
    va_start(args, format);
    vprint(format, args);
    va_end(args);
}
