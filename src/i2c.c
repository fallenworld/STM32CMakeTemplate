/*
 * STM32 I2C helper functions.
 */

#include "stm32.h"

static void i2c_pin_write(const struct gpio_pin *pin, uint8_t bit)
{
    gpio_pin_write(pin, bit);
    delay_us(10);
}

static uint8_t i2c_pin_read(const struct gpio_pin *pin)
{
    uint8_t data = gpio_pin_read(pin);
    delay_us(10);
    return data;
}

bool i2c_software_init(const struct i2c_software *i2c)
{
    if (!gpio_pin_init(&i2c->scl, GPIO_Mode_Out_OD)
        || !gpio_pin_init(&i2c->sda, GPIO_Mode_Out_OD))
        return false;

    gpio_pin_write(&i2c->scl, 1);
    gpio_pin_write(&i2c->sda, 1);

    TRACE("Inited software I2C.\n");

    return true;
}

void i2c_software_start(const struct i2c_software *i2c)
{
    i2c_pin_write(&i2c->sda, 1);
    i2c_pin_write(&i2c->scl, 1);
    i2c_pin_write(&i2c->sda, 0);
    i2c_pin_write(&i2c->scl, 0);
}

void i2c_software_stop(const struct i2c_software *i2c)
{
    i2c_pin_write(&i2c->sda, 0);
    i2c_pin_write(&i2c->scl, 1);
    i2c_pin_write(&i2c->sda, 1);
}

void i2c_software_send(const struct i2c_software *i2c, uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; ++i)
    {
        i2c_pin_write(&i2c->sda, !!(data & (0x80 >> i)));
        i2c_pin_write(&i2c->scl, 1);
        i2c_pin_write(&i2c->scl, 0);
    }
}

uint8_t i2c_software_receive(const struct i2c_software *i2c)
{
    uint8_t data = 0, i;

    i2c_pin_write(&i2c->sda, 1);

    for (i = 0; i < 8; ++i)
    {
        i2c_pin_write(&i2c->scl, 1);
        if (i2c_pin_read(&i2c->sda))
            data |= (0x80 >> i);
        i2c_pin_write(&i2c->scl, 0);
    }

    return data;
}

void i2c_software_send_ack(const struct i2c_software *i2c, uint8_t ack_bit)
{
    i2c_pin_write(&i2c->sda, ack_bit);
    i2c_pin_write(&i2c->scl, 1);
    i2c_pin_write(&i2c->scl, 0);
}

uint8_t i2c_software_receive_ack(const struct i2c_software *i2c)
{
    uint8_t ack_bit;
    i2c_pin_write(&i2c->sda, 1);
    i2c_pin_write(&i2c->scl, 1);
    ack_bit = i2c_pin_read(&i2c->sda);
    i2c_pin_write(&i2c->scl, 0);
    return ack_bit;
}
