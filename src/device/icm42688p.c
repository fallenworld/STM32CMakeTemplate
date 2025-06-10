/*
 * ICM-42688-P driver.
 * Datasheet: https://invensense.tdk.com/download-pdf/icm-42688-p-datasheet
 */

#include "icm42688p.h"

static bool icm42688p_config_accel(struct i2c_software_device *device, uint8_t odr, uint8_t fs)
{
    return icm42688p_write_byte(device, ICM42688P_REG_ACCEL_CONFIG0, (fs << 5) | odr);
}

static bool icm42688p_config_gyro(struct i2c_software_device *device, uint8_t odr, uint8_t fs)
{
    return icm42688p_write_byte(device, ICM42688P_REG_GYRO_CONFIG0, (fs << 5) | odr);
}

bool icm42688p_init(struct i2c_software_device *device, uint8_t ad0)
{
    uint8_t id = 0;

    if (!i2c_software_init(&device->i2c))
        return false;

    device->address = ICM42688P_I2C_ADDRESS | ad0;

    /* Check device ID. */
    if (!icm42688p_read_byte(device, ICM42688P_REG_WHO_AM_I, &id) || id != ICM42688P_ID)
    {
        TRACE("Invalid device ID %#x.\n", id);
        return false;
    }

    /* Reset device. */
    if (!icm42688p_write_byte(device, ICM42688P_REG_PWR_MGMT0, 0))
    {
        TRACE("Failed to reset device.\n");
        return false;
    }
    delay_ms(10);

    /* Config accelerometer. */
    if (!icm42688p_config_accel(device, ICM42688P_ODR_1KHZ, ICM42688P_AFS_16G))
    {
        TRACE("Failed to config accelerometer.\n");
        return false;
    }

    /* Config gyroscope. */
    if (!icm42688p_config_gyro(device, ICM42688P_ODR_1KHZ, ICM42688P_GYRO_FS_2000DPS))
    {
        TRACE("Failed to config gyroscope.\n");
        return false;
    }

    /* Activate device. [3:2] GYRO_MODE, [1:0] ACCEL_MODE.
     * Place accelerometer and gyroscope in low noice mode. */
    if (!icm42688p_write_byte(device, ICM42688P_REG_PWR_MGMT0, 0x0f))
    {
        TRACE("Failed to activate device.\n");
        return false;
    }

    TRACE("Inited ICM-42688-P.\n");

    return true;
}

bool icm42688p_write(const struct i2c_software_device *device,
        uint8_t reg_addr, uint8_t *data, uint32_t size)
{
    const struct i2c_software *i2c = &device->i2c;
    uint32_t i;

    i2c_software_start(i2c);

    /* Send device address. */
    i2c_software_send(i2c, I2C_W(device->address));
    if (i2c_software_receive_ack(i2c) != I2C_ACK)
    {
        i2c_software_stop(i2c);
        TRACE("Failed to send device address.\n");
        return false;
    }

    /* Send register address. */
    i2c_software_send(i2c, reg_addr);
    if (i2c_software_receive_ack(i2c) != I2C_ACK)
    {
        i2c_software_stop(i2c);
        TRACE("Failed to send register address.\n");
        return false;
    }

    /* Send data. */
    for (i = 0; i < size; ++i)
    {
        i2c_software_send(i2c, data[i]);
        if (i2c_software_receive_ack(i2c) != I2C_ACK)
        {
            i2c_software_stop(i2c);
            TRACE("Failed to send data[%zu].\n", i);
            return false;
        }
    }

    i2c_software_stop(i2c);

    TRACE("Wrote %u bytes to %#x, device %#x.\n", size, reg_addr, device->address);
    return true;
}

bool icm42688p_read(const struct i2c_software_device *device,
        uint8_t reg_addr, uint8_t *data, uint32_t size)
{
    const struct i2c_software *i2c = &device->i2c;
    uint32_t i;

    i2c_software_start(i2c);

    /* Send device address for write. */
    i2c_software_send(i2c, I2C_W(device->address));
    if (i2c_software_receive_ack(i2c) != I2C_ACK)
    {
        i2c_software_stop(i2c);
        TRACE("Failed to send device address for write.\n");
        return false;
    }

    /* Send register address. */
    i2c_software_send(i2c, reg_addr);
    if (i2c_software_receive_ack(i2c) != I2C_ACK)
    {
        i2c_software_stop(i2c);
        TRACE("Failed to send register address.\n");
        return false;
    }

    i2c_software_start(i2c);

    /* Send device address for read. */
    i2c_software_send(i2c, I2C_R(device->address));
    if (i2c_software_receive_ack(i2c) != I2C_ACK)
    {
        i2c_software_stop(i2c);
        TRACE("Failed to send device address for read.\n");
        return false;
    }

    /* Read data. */
    for (i = 0; i < size; ++i)
    {
        data[i] = i2c_software_receive(i2c);
        i2c_software_send_ack(i2c, i == size - 1 ? I2C_NACK : I2C_ACK);
    }

    i2c_software_stop(i2c);

    /* TRACE("Read %u bytes from %#x, device %#x.\n", size, reg_addr, device->address); */
    return true;
}

bool icm42688p_write_byte(const struct i2c_software_device *device, uint8_t reg_addr, uint8_t byte)
{
    return icm42688p_write(device, reg_addr, &byte, 1);
}

bool icm42688p_read_byte(const struct i2c_software_device *device, uint8_t reg_addr, uint8_t *byte)
{
    return icm42688p_read(device, reg_addr, byte, 1);
}

void icm42688p_get_accel(const struct i2c_software_device *device,
        int16_t *accel_x, int16_t *accel_y, int16_t *accel_z)
{
    if (accel_x)
    {
        icm42688p_read_byte(device, ICM42688P_REG_ACCEL_DATA_X0, (uint8_t *)accel_x);
        icm42688p_read_byte(device, ICM42688P_REG_ACCEL_DATA_X1, (uint8_t *)accel_x + 1);
    }

    if (accel_y)
    {
        icm42688p_read_byte(device, ICM42688P_REG_ACCEL_DATA_Y0, (uint8_t *)accel_y);
        icm42688p_read_byte(device, ICM42688P_REG_ACCEL_DATA_Y1, (uint8_t *)accel_y + 1);
    }

    if (accel_z)
    {
        icm42688p_read_byte(device, ICM42688P_REG_ACCEL_DATA_Z0, (uint8_t *)accel_z);
        icm42688p_read_byte(device, ICM42688P_REG_ACCEL_DATA_Z1, (uint8_t *)accel_z + 1);
    }
}

void icm42688p_get_gero(const struct i2c_software_device *device,
        int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    if (gyro_x)
    {
        icm42688p_read_byte(device, ICM42688P_REG_GYRO_DATA_X0, (uint8_t *) gyro_x);
        icm42688p_read_byte(device, ICM42688P_REG_GYRO_DATA_X1, (uint8_t *) gyro_x + 1);
    }

    if (gyro_y)
    {
        icm42688p_read_byte(device, ICM42688P_REG_GYRO_DATA_Y0, (uint8_t *) gyro_y);
        icm42688p_read_byte(device, ICM42688P_REG_GYRO_DATA_Y1, (uint8_t *) gyro_y + 1);
    }

    if (gyro_z)
    {
        icm42688p_read_byte(device, ICM42688P_REG_GYRO_DATA_Z0, (uint8_t *) gyro_z);
        icm42688p_read_byte(device, ICM42688P_REG_GYRO_DATA_Z1, (uint8_t *) gyro_z + 1);
    }
}
