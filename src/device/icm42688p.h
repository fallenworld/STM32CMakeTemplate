/*
 * ICM-42688-P header.
 */

#ifndef __ICM_42688_P_H__
#define __ICM_42688_P_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32.h"

/* Registers. */
#define ICM42688P_REG_DEVICE_CONFIG        0x11
#define ICM42688P_REG_DRIVE_CONFIG         0x13
#define ICM42688P_REG_INT_CONFIG           0x14
#define ICM42688P_REG_FIFO_CONFIG          0x16
#define ICM42688P_REG_TEMP_DATA1           0x1d
#define ICM42688P_REG_TEMP_DATA0           0x1e
#define ICM42688P_REG_ACCEL_DATA_X1        0x1f
#define ICM42688P_REG_ACCEL_DATA_X0        0x20
#define ICM42688P_REG_ACCEL_DATA_Y1        0x21
#define ICM42688P_REG_ACCEL_DATA_Y0        0x22
#define ICM42688P_REG_ACCEL_DATA_Z1        0x23
#define ICM42688P_REG_ACCEL_DATA_Z0        0x24
#define ICM42688P_REG_GYRO_DATA_X1         0x25
#define ICM42688P_REG_GYRO_DATA_X0         0x26
#define ICM42688P_REG_GYRO_DATA_Y1         0x27
#define ICM42688P_REG_GYRO_DATA_Y0         0x28
#define ICM42688P_REG_GYRO_DATA_Z1         0x29
#define ICM42688P_REG_GYRO_DATA_Z0         0x2a
#define ICM42688P_REG_TMST_FSYNCH          0x2b
#define ICM42688P_REG_TMST_FSYNCL          0x2c
#define ICM42688P_REG_INT_STATUS           0x2d
#define ICM42688P_REG_FIFO_COUNTH          0x2e
#define ICM42688P_REG_FIFO_COUNTL          0x2f
#define ICM42688P_REG_FIFO_DATA            0x30
#define ICM42688P_REG_APEX_DATA0           0x31
#define ICM42688P_REG_APEX_DATA1           0x32
#define ICM42688P_REG_APEX_DATA2           0x33
#define ICM42688P_REG_APEX_DATA3           0x34
#define ICM42688P_REG_APEX_DATA4           0x35
#define ICM42688P_REG_APEX_DATA5           0x36
#define ICM42688P_REG_INT_STATUS2          0x37
#define ICM42688P_REG_INT_STATUS3          0x38
#define ICM42688P_REG_SIGNAL_PATH_RESET    0x4b
#define ICM42688P_REG_INTF_CONFIG0         0x4c
#define ICM42688P_REG_INTF_CONFIG1         0x4d
#define ICM42688P_REG_PWR_MGMT0            0x4e
#define ICM42688P_REG_GYRO_CONFIG0         0x4f
#define ICM42688P_REG_ACCEL_CONFIG0        0x50
#define ICM42688P_REG_GYRO_CONFIG1         0x51
#define ICM42688P_REG_GYRO_ACCEL_CONFIG0   0x52
#define ICM42688P_REG_ACCEL_CONFIG1        0x53
#define ICM42688P_REG_TMST_CONFIG          0x54
#define ICM42688P_REG_APEX_CONFIG0         0x56
#define ICM42688P_REG_SMD_CONFIG           0x57
#define ICM42688P_REG_FIFO_CONFIG1         0x5f
#define ICM42688P_REG_FIFO_CONFIG2         0x60
#define ICM42688P_REG_FIFO_CONFIG3         0x61
#define ICM42688P_REG_FSYNC_CONFIG         0x62
#define ICM42688P_REG_INT_CONFIG0          0x63
#define ICM42688P_REG_INT_CONFIG1          0x64
#define ICM42688P_REG_INT_SOURCE0          0x65
#define ICM42688P_REG_INT_SOURCE1          0x66
#define ICM42688P_REG_INT_SOURCE3          0x68
#define ICM42688P_REG_INT_SOURCE4          0x69
#define ICM42688P_REG_FIFO_LOST_PKT0       0x6c
#define ICM42688P_REG_FIFO_LOST_PKT1       0x6d
#define ICM42688P_REG_SELF_TEST_CONFIG     0x70
#define ICM42688P_REG_WHO_AM_I             0x75
#define ICM42688P_REG_REG_BANK_SEL         0x76
#define ICM42688P_REG_SENSOR_CONFIG0       0x03
#define ICM42688P_REG_GYRO_CONFIG_STATIC2  0x0b
#define ICM42688P_REG_GYRO_CONFIG_STATIC3  0x0c
#define ICM42688P_REG_GYRO_CONFIG_STATIC4  0x0d
#define ICM42688P_REG_GYRO_CONFIG_STATIC5  0x0e
#define ICM42688P_REG_GYRO_CONFIG_STATIC6  0x0f
#define ICM42688P_REG_GYRO_CONFIG_STATIC7  0x10
#define ICM42688P_REG_GYRO_CONFIG_STATIC8  0x11
#define ICM42688P_REG_GYRO_CONFIG_STATIC9  0x12
#define ICM42688P_REG_GYRO_CONFIG_STATIC10 0x13
#define ICM42688P_REG_XG_ST_DATA           0x5f
#define ICM42688P_REG_YG_ST_DATA           0x60
#define ICM42688P_REG_ZG_ST_DATA           0x61
#define ICM42688P_REG_TMSTVAL0             0x62
#define ICM42688P_REG_TMSTVAL1             0x63
#define ICM42688P_REG_TMSTVAL2             0x64
#define ICM42688P_REG_INTF_CONFIG4         0x7a
#define ICM42688P_REG_INTF_CONFIG5         0x7b
#define ICM42688P_REG_INTF_CONFIG6         0x7c

/* Gyroscope ODR. */
#define ICM42688P_ODR_32KHZ    0x1
#define ICM42688P_ODR_16KHZ    0x2
#define ICM42688P_ODR_8KHZ     0x3
#define ICM42688P_ODR_4KHZ     0x4
#define ICM42688P_ODR_2KHZ     0x5
#define ICM42688P_ODR_1KHZ     0x6
#define ICM42688P_ODR_200HZ    0x7
#define ICM42688P_ODR_100HZ    0x8
#define ICM42688P_ODR_50HZ     0x9
#define ICM42688P_ODR_25HZ     0xa
#define ICM42688P_ODR_12_5HZ   0xb
#define ICM42688P_ODR_6_25HZ   0xc
#define ICM42688P_ODR_3_125HZ  0xd
#define ICM42688P_ODR_1_5625HZ 0xe
#define ICM42688P_ODR_500HZ    0xf

/* Full scale for accelerometer. */
#define ICM42688P_AFS_16G 0
#define ICM42688P_AFS_8G  1
#define ICM42688P_AFS_4G  2
#define ICM42688P_AFS_2G  3

/* Full scale for gyroscope. */
#define ICM42688P_GYRO_FS_2000DPS   0
#define ICM42688P_GYRO_FS_1000DPS   1
#define ICM42688P_GYRO_FS_500DPS    2
#define ICM42688P_GYRO_FS_250DPS    3
#define ICM42688P_GYRO_FS_125DPS    4
#define ICM42688P_GYRO_FS_62_5DPS   5
#define ICM42688P_GYRO_FS_31_25DPS  6
#define ICM42688P_GYRO_FS_15_625DPS 7

#define ICM42688P_ID 0x47

#define ICM42688P_I2C_ADDRESS 0x68

/* Functions. */
bool icm42688p_init(struct i2c_device *device, uint8_t ad0, bool software_i2c);
bool icm42688p_read(const struct i2c_device *device, uint8_t reg_addr, uint8_t *data, uint32_t size);
bool icm42688p_write(const struct i2c_device *device, uint8_t reg_addr, uint8_t *data, uint32_t size);
bool icm42688p_read_byte(const struct i2c_device *device, uint8_t reg_addr, uint8_t *byte);
bool icm42688p_write_byte(const struct i2c_device *device, uint8_t reg_addr, uint8_t byte);
void icm42688p_get_accel(const struct i2c_device *device,
        int16_t *accel_x, int16_t *accel_y, int16_t *accel_z);
void icm42688p_get_gero(const struct i2c_device *device,
        int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#endif /* __ICM_42688_P_H__ */
