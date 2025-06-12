/*
 * STM32 I2C helper functions.
 */

#include "stm32.h"

#define I2C_HARDWARE_TIMEOUT 10000

struct i2c_hardware_info
{
    I2C_TypeDef *i2c;
    uint32_t periph;
    struct gpio_pin scl, sda;
};

static const struct i2c_hardware_info i2c_hardware_info_list[] =
{
    {I2C1, RCC_APB1Periph_I2C1, {GPIOB, GPIO_Pin_6},  {GPIOB, GPIO_Pin_7}},
    {I2C2, RCC_APB1Periph_I2C2, {GPIOB, GPIO_Pin_10}, {GPIOB, GPIO_Pin_11}},
};

static const struct i2c_hardware_info *i2c_hardware_info_find(I2C_TypeDef *i2c)
{
    if (i2c == I2C1)
        return &i2c_hardware_info_list[0];
    if (i2c == I2C2)
        return &i2c_hardware_info_list[1];
    return NULL;
}

static void i2c_software_pin_write(const struct gpio_pin *pin, uint8_t bit)
{
    gpio_pin_write(pin, bit);
    delay_us(10);
}

static uint8_t i2c_software_pin_read(const struct gpio_pin *pin)
{
    uint8_t data = gpio_pin_read(pin);
    delay_us(10);
    return data;
}

static void i2c_software_send_ack(const struct i2c *i2c, uint8_t ack_bit)
{
    i2c_software_pin_write(&i2c->software.sda, ack_bit);
    i2c_software_pin_write(&i2c->software.scl, 1);
    i2c_software_pin_write(&i2c->software.scl, 0);
}

static uint8_t i2c_software_receive_ack(const struct i2c *i2c)
{
    uint8_t ack_bit;
    i2c_software_pin_write(&i2c->software.sda, 1);
    i2c_software_pin_write(&i2c->software.scl, 1);
    ack_bit = i2c_software_pin_read(&i2c->software.sda);
    i2c_software_pin_write(&i2c->software.scl, 0);
    return ack_bit;
}

static void i2c_software_start(const struct i2c *i2c)
{
    i2c_software_pin_write(&i2c->software.sda, 1);
    i2c_software_pin_write(&i2c->software.scl, 1);
    i2c_software_pin_write(&i2c->software.sda, 0);
    i2c_software_pin_write(&i2c->software.scl, 0);
}

static void i2c_software_stop(const struct i2c *i2c)
{
    i2c_software_pin_write(&i2c->software.sda, 0);
    i2c_software_pin_write(&i2c->software.scl, 1);
    i2c_software_pin_write(&i2c->software.sda, 1);
}

static bool i2c_software_send(const struct i2c *i2c, uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; ++i)
    {
        i2c_software_pin_write(&i2c->software.sda, !!(data & (0x80 >> i)));
        i2c_software_pin_write(&i2c->software.scl, 1);
        i2c_software_pin_write(&i2c->software.scl, 0);
    }

    return i2c_software_receive_ack(i2c) == I2C_ACK;
}

static uint8_t i2c_software_receive(const struct i2c *i2c, bool stop)
{
    uint8_t data = 0, i;

    i2c_software_pin_write(&i2c->software.sda, 1);

    for (i = 0; i < 8; ++i)
    {
        i2c_software_pin_write(&i2c->software.scl, 1);
        if (i2c_software_pin_read(&i2c->software.sda))
            data |= (0x80 >> i);
        i2c_software_pin_write(&i2c->software.scl, 0);
    }

    i2c_software_send_ack(i2c, stop ? I2C_NACK : I2C_ACK);

    if (stop)
        i2c_software_stop(i2c);

    return data;
}

static bool i2c_software_send_address(const struct i2c *i2c, uint8_t address, uint8_t rw)
{
    return i2c_software_send(i2c, (address << 1) | rw);
}

static const struct i2c_ops i2c_software_ops =
{
    i2c_software_start,
    i2c_software_stop,
    i2c_software_send,
    i2c_software_receive,
    i2c_software_send_address,
};

bool i2c_software_init(struct i2c *i2c)
{
    if (!gpio_pin_init(&i2c->software.scl, GPIO_Mode_Out_OD)
            || !gpio_pin_init(&i2c->software.sda, GPIO_Mode_Out_OD))
        return false;

    i2c->ops = &i2c_software_ops;

    gpio_pin_write(&i2c->software.scl, 1);
    gpio_pin_write(&i2c->software.sda, 1);

    TRACE("Inited software I2C.\n");

    return true;
}

static bool i2c_hardware_wait_event(const struct i2c *i2c, uint32_t event, uint32_t timeout_count)
{
    uint32_t timeout = timeout_count;

    while (I2C_CheckEvent(i2c->hardware, event) != SUCCESS)
    {
        if (!timeout--)
        {
            TRACE("Timeout waiting for I2C event %#x.\n", event);
            return false;
        }
    }

    return true;
}

static void i2c_hardware_start(const struct i2c *i2c)
{
    I2C_GenerateSTART(i2c->hardware, ENABLE);
    i2c_hardware_wait_event(i2c, I2C_EVENT_MASTER_MODE_SELECT, I2C_HARDWARE_TIMEOUT);
}

static void i2c_hardware_stop(const struct i2c *i2c)
{
    I2C_GenerateSTOP(i2c->hardware, ENABLE);
}

static bool i2c_hardware_send(const struct i2c *i2c, uint8_t data)
{
    /* We are using I2C_EVENT_MASTER_BYTE_TRANSMITTED, which is
     * slower than I2C_EVENT_MASTER_BYTE_TRANSMITTING but more reliable.*/
    I2C_SendData(i2c->hardware, data);
    return i2c_hardware_wait_event(i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_HARDWARE_TIMEOUT);
}

static uint8_t i2c_hardware_receive(const struct i2c *i2c, bool stop)
{
    if (stop)
    {
        I2C_AcknowledgeConfig(i2c->hardware, DISABLE);
        i2c_hardware_stop(i2c);
    }
    else
    {
        I2C_AcknowledgeConfig(i2c->hardware, ENABLE);
    }

    if (!i2c_hardware_wait_event(i2c, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_HARDWARE_TIMEOUT))
        return 0;

    return I2C_ReceiveData(i2c->hardware);
}

static bool i2c_hardware_send_address(const struct i2c *i2c, uint8_t address, uint8_t rw)
{
    uint32_t event = (rw == I2C_W) ?
            I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED :
            I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED;

    I2C_Send7bitAddress(i2c->hardware, address << 1, rw);
    return i2c_hardware_wait_event(i2c, event, I2C_HARDWARE_TIMEOUT);
}

static const struct i2c_ops i2c_hardware_ops =
{
    i2c_hardware_start,
    i2c_hardware_stop,
    i2c_hardware_send,
    i2c_hardware_receive,
    i2c_hardware_send_address,
};

bool i2c_hardware_init(struct i2c *i2c)
{
    const struct i2c_hardware_info *info = i2c_hardware_info_find(i2c->hardware);
    I2C_InitTypeDef i2c_init_struct;

    if (!info)
    {
        TRACE("Invalid I2C %p.\n", i2c->hardware);
        return false;
    }

    if (!gpio_pin_init(&info->scl, GPIO_Mode_AF_OD)
            || !gpio_pin_init(&info->sda, GPIO_Mode_AF_OD))
        return false;

    abp1_periph_enable(info->periph);

    i2c->ops = &i2c_hardware_ops;

    I2C_StructInit(&i2c_init_struct);
    i2c_init_struct.I2C_ClockSpeed = 50000;
    i2c_init_struct.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c_init_struct.I2C_Ack = I2C_Ack_Enable;
    I2C_Init(i2c->hardware, &i2c_init_struct);

    I2C_Cmd(i2c->hardware, ENABLE);

    TRACE("Inited hardware I2C.\n");

    return true;
}
