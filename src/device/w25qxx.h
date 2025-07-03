/*
 * W25QXX header.
 */

#ifndef __W25QXX_H__
#define __W25QXX_H__

#include "stm32.h"

#define W25QXX_INSTR_WRITE_ENABLE            0x06
#define W25QXX_INSTR_VOLATILE_SR_WRITE_EN    0x50
#define W25QXX_INSTR_WRITE_DISABLE           0x04
#define W25QXX_INSTR_RELEASE_POWERDOWN_ID    0xab
#define W25QXX_INSTR_MANUFACTURER_ID         0x90
#define W25QXX_INSTR_JEDEC_ID                0x9f
#define W25QXX_INSTR_READ_UNIQUE_ID          0x4b
#define W25QXX_INSTR_READ_DATA               0x03
#define W25QXX_INSTR_FAST_READ               0x0b
#define W25QXX_INSTR_PAGE_PROGRAM            0x02
#define W25QXX_INSTR_SECTOR_ERASE_4KB        0x20
#define W25QXX_INSTR_BLOCK_ERASE_32KB        0x52
#define W25QXX_INSTR_BLOCK_ERASE_64KB        0xd8
#define W25QXX_INSTR_CHIP_ERASE              0xc7
#define W25QXX_INSTR_READ_STATUS_REG1        0x05
#define W25QXX_INSTR_WRITE_STATUS_REG1       0x01
#define W25QXX_INSTR_READ_STATUS_REG2        0x35
#define W25QXX_INSTR_WRITE_STATUS_REG2       0x31
#define W25QXX_INSTR_READ_STATUS_REG3        0x15
#define W25QXX_INSTR_WRITE_STATUS_REG3       0x11
#define W25QXX_INSTR_READ_SFDP               0x5a
#define W25QXX_INSTR_ERASE_SECURITY_REG      0x44
#define W25QXX_INSTR_PROGRAM_SECURITY_REG    0x42
#define W25QXX_INSTR_READ_SECURITY_REG       0x48
#define W25QXX_INSTR_GLOBAL_BLOCK_LOCK       0x7e
#define W25QXX_INSTR_GLOBAL_BLOCK_UNLOCK     0x98
#define W25QXX_INSTR_READ_BLOCK_LOCK         0x3d
#define W25QXX_INSTR_INDIVIDUAL_BLOCK_LOCK   0x36
#define W25QXX_INSTR_INDIVIDUAL_BLOCK_UNLOCK 0x39
#define W25QXX_INSTR_ERASE_PROGRAM_SUSPEND   0x75
#define W25QXX_INSTR_ERASE_PROGRAM_RESUME    0x7a
#define W25QXX_INSTR_POWER_DOWN              0xb9
#define W25QXX_INSTR_ENABLE_RESET            0x66
#define W25QXX_INSTR_RESET_DEVICE            0x99

#define W25QXX_MANUFACTURER_ID 0xef
#define W25QXX_DEVICE_ID       0x4017

#define W25QXX_STATUS_BUSY  1
#define W25QXX_STATUS_READY 0

bool w25qxx_init(struct device *spi);
void w25qxx_read(struct device *spi, uint32_t addr, void *data, uint32_t size);
void w25qxx_write(struct device *spi, uint32_t addr, const void *data, uint32_t size);
void w25qxx_erase(struct device *spi, uint8_t erase_type, uint32_t addr);

#endif /* __W25QXX_H__ */
