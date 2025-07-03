/*
 * STM32 helper functions header.
 */

#ifndef __STM32_H__
#define __STM32_H__

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"

#ifndef __PROJECT_NAME__
#define __PROJECT_NAME__ "STM32"
#endif

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

typedef void (*irq_handler)(void);

struct i2c;

struct gpio_pin
{
    GPIO_TypeDef *gpio;
    uint16_t pin;
};

struct i2c_software
{
    struct gpio_pin scl, sda;
};

struct i2c_ops
{
    void (*i2c_start)(const struct i2c *i2c);
    void (*i2c_stop)(const struct i2c *i2c);
    bool (*i2c_send)(const struct i2c *i2c, uint8_t data);
    uint8_t (*i2c_receive)(const struct i2c *i2c, bool stop);
    bool (*i2c_send_address)(const struct i2c *i2c, uint8_t address, uint8_t rw);
};

struct i2c
{
    union
    {
        struct i2c_software software;
        I2C_TypeDef *hardware;
    };
    const struct i2c_ops *ops;
};

struct i2c_device
{
    struct i2c i2c;
    uint8_t address;
};

struct device
{
    uint8_t type;
};

/* GPIO. */
bool gpio_init(GPIO_TypeDef *gpio, uint16_t pins, GPIOMode_TypeDef mode);
bool gpio_pin_init(const struct gpio_pin *pin, GPIOMode_TypeDef mode);
uint8_t gpio_pin_read(const struct gpio_pin *pin);
void gpio_pin_write(const struct gpio_pin *pin, uint8_t bit);
bool exti_init(GPIO_TypeDef *gpio, uint16_t pin, irq_handler handler,
        EXTITrigger_TypeDef trigger, uint8_t preemption_pri, uint8_t sub_pri);

/* USART. */
const struct usart_info *usart_info_find(const USART_TypeDef *usart);
const char *usart_name(const USART_TypeDef *usart);
bool usart_init(USART_TypeDef *usart,
        uint32_t baud_rate, uint16_t word_length, uint16_t stop_bits, uint16_t parity, irq_handler receive_handler);
bool usart_simple_init(USART_TypeDef *usart);
void usart_send_byte(USART_TypeDef *usart, uint8_t byte);
void usart_send_str(USART_TypeDef *usart, const char *str);
bool usart_has_data(USART_TypeDef *usart);
uint8_t usart_wait_byte(USART_TypeDef *usart);

/* Timer. */
bool timer_update_init(TIM_TypeDef *timer, bool internal_clock, uint32_t prescaler_factor, uint32_t period_count);
bool timer_pwm_init(TIM_TypeDef *timer, uint16_t channel, uint32_t frequency, uint32_t period_count);
bool timer_pwm_set_pulse(TIM_TypeDef *timer, uint16_t channel, uint16_t pulse);
bool timer_input_capture_init(TIM_TypeDef *timer, uint16_t channel, bool use_pwmi);
uint32_t timer_input_capture_get_frequency(TIM_TypeDef *timer, uint16_t channel);
uint16_t timer_input_capture_get_duty(TIM_TypeDef *timer, uint16_t channel);
bool encoder_init(TIM_TypeDef *timer);
uint16_t encoder_get_count(TIM_TypeDef *timer);
bool servo_init(TIM_TypeDef *timer, uint16_t channel);
bool servo_set_angle(TIM_TypeDef *timer, uint16_t channel, uint32_t angle);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/* ADC. */
bool adc_single_init(ADC_TypeDef *adc);
bool adc_dma_init(ADC_TypeDef *adc, uint8_t adc_channel_count, void *dst_addr);
bool adc_init_channel(ADC_TypeDef *adc, uint8_t channel);
void adc_start_single_convert(ADC_TypeDef *adc, uint8_t channel);
uint16_t adc_wait_value(ADC_TypeDef *adc);

/* DMA. */
bool dma_init(DMA_Channel_TypeDef *channel,
        void *src_addr, void *dst_addr, uint32_t data_size, bool m2m, bool periph_inc, bool circular);
bool dma_m2m_init(DMA_Channel_TypeDef *channel, void *src_addr, void *dst_addr, uint32_t data_size);
void dma_start_transfer(DMA_Channel_TypeDef *channel, uint16_t data_count);
void dma_wait_transfer(DMA_Channel_TypeDef *channel);

/* I2C. */
#define I2C_ACK  0
#define I2C_NACK 1

#define I2C_W I2C_Direction_Transmitter
#define I2C_R I2C_Direction_Receiver

#define i2c_start(i2c)                     ((i2c)->ops->i2c_start(i2c))
#define i2c_stop(i2c)                      ((i2c)->ops->i2c_stop(i2c))
#define i2c_send(i2c, data)                ((i2c)->ops->i2c_send(i2c, data))
#define i2c_receive(i2c, stop)             ((i2c)->ops->i2c_receive(i2c, stop))
#define i2c_send_address(i2c, address, rw) ((i2c)->ops->i2c_send_address(i2c, address, rw))

bool i2c_software_init(struct i2c *i2c);
bool i2c_hardware_init(struct i2c *i2c);

/* SPI. */
#define SPI_SOFTWARE 0
#define SPI_HARDWARE 1
#define SPI_DUMMY_BYTE 0xff

struct spi_software
{
    struct device device;
    struct gpio_pin ss, sck, miso, mosi;
};

struct spi_hardware
{
    struct device device;
    SPI_TypeDef *hardware;
};

bool spi_init(struct device *spi);
void spi_start(struct device *spi);
void spi_stop(struct device *spi);
uint8_t spi_read_write(struct device *spi, uint8_t data);

/* Interrupt handlers. */
bool exti_set_handler(uint32_t exti_line, irq_handler handler);
bool usart_set_handler(USART_TypeDef *usart, irq_handler handler);

/* Debug. */
bool debug_init(USART_TypeDef *usart, GPIO_TypeDef *debug_led_gpio, uint16_t debug_led_pin);
void debug_trace(const char *file, int line, const char *func, const char *format, ...);
const char *debug_buffer_vsprintf(const char *format, va_list args);
const char *debug_buffer_sprintf(const char *format, ...);
void __assert_func(const char *file, int line, const char *func, const char *expr);
void assert_failed(uint8_t *file, uint32_t line);

#ifdef DEBUG
#define TRACE(format, ...) debug_trace(__FILE_NAME__, __LINE__, __func__, format, ##__VA_ARGS__)
#else
#define TRACE
#endif /* DEBUG */

#if defined(__GNUC__) || defined(__clang__)
#define count_trailing_zeros(x) __builtin_ctz(x)
#else
static inline uint32_t count_trailing_zeros(uint32_t x)
{
    uint32_t n = 0;
    if (x == 0)
        return 32;
    if ((x & 0x0000FFFF) == 0) { n += 16; x >>= 16; }
    if ((x & 0x000000FF) == 0) { n += 8;  x >>= 8;  }
    if ((x & 0x0000000F) == 0) { n += 4;  x >>= 4;  }
    if ((x & 0x00000003) == 0) { n += 2;  x >>= 2;  }
    if ((x & 0x00000001) == 0) { n += 1;            }
    return n;
}
#endif

static inline bool is_abp1_periph_enabled(uint32_t periph)
{
    return !!(RCC->APB1ENR & periph);
}

static inline bool is_abp2_periph_enabled(uint32_t periph)
{
    return !!(RCC->APB2ENR & periph);
}

static inline bool is_ahb_periph_enabled(uint32_t periph)
{
    return !!(RCC->AHBENR & periph);
}

static inline void abp1_periph_enable(uint32_t periph)
{
    if (!is_abp1_periph_enabled(periph))
        RCC_APB1PeriphClockCmd(periph, ENABLE);
}

static inline void abp2_periph_enable(uint32_t periph)
{
    if (!is_abp2_periph_enabled(periph))
        RCC_APB2PeriphClockCmd(periph, ENABLE);
}

static inline void ahb_periph_enable(uint32_t periph)
{
    if (!is_ahb_periph_enabled(periph))
        RCC_AHBPeriphClockCmd(periph, ENABLE);
}

#endif /* __STM32_H__ */
