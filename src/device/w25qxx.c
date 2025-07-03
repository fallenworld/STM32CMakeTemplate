/*
 * W25QXX driver.
 */

#include "w25qxx.h"

#define WAIT_TIMEOUT 10000

/* Send a 24-bits address. */
static void w25qxx_send_address(struct device *spi, uint32_t addr)
{
    spi_read_write(spi, addr >> 16);
    spi_read_write(spi, addr >> 8);
    spi_read_write(spi, addr);
}

static void w25qxx_get_id(struct device *spi, uint8_t *mid, uint16_t *did)
{
    spi_start(spi);
    spi_read_write(spi, W25QXX_INSTR_JEDEC_ID);
    *mid = spi_read_write(spi, SPI_DUMMY_BYTE);
    *did = spi_read_write(spi, SPI_DUMMY_BYTE);
    *did <<= 8;
    *did |= spi_read_write(spi, SPI_DUMMY_BYTE);
    spi_stop(spi);
}

static void w25qxx_write_enable(struct device *spi)
{
    spi_start(spi);
    spi_read_write(spi, W25QXX_INSTR_WRITE_ENABLE);
    spi_stop(spi);
}

static void w25qxx_wait(struct device *spi, uint32_t timeout_count)
{
    uint8_t status;

    spi_start(spi);
    spi_read_write(spi, W25QXX_INSTR_READ_STATUS_REG1);

    while (((status = spi_read_write(spi, SPI_DUMMY_BYTE)) & 0x1) != W25QXX_STATUS_READY)
    {
        if (!timeout_count--)
        {
            TRACE("Timeout waiting for SPI.\n");
            break;
        }
    }

    spi_stop(spi);
}

bool w25qxx_init(struct device *spi)
{
    uint16_t did;
    uint8_t mid;

    if (!spi_init(spi))
        return false;

    w25qxx_get_id(spi, &mid, &did);
    if (mid != W25QXX_MANUFACTURER_ID || did != W25QXX_DEVICE_ID)
    {
        TRACE("Invalid JEDEC ID: Manufacturer ID %#x, device ID %#x.\n", mid, did);
        return false;
    }

    TRACE("Inited W25QXX.\n");

    return true;
}

void w25qxx_read(struct device *spi, uint32_t addr, void *data, uint32_t size)
{
    uint32_t i;
    spi_start(spi);
    spi_read_write(spi, W25QXX_INSTR_READ_DATA);
    w25qxx_send_address(spi, addr);
    for (i = 0; i < size; ++i)
        ((uint8_t *)data)[i] = spi_read_write(spi, SPI_DUMMY_BYTE);
    spi_stop(spi);
}

void w25qxx_write(struct device *spi, uint32_t addr, const void *data, uint32_t size)
{
    uint32_t i;

    w25qxx_write_enable(spi);

    spi_start(spi);
    spi_read_write(spi, W25QXX_INSTR_PAGE_PROGRAM);
    w25qxx_send_address(spi, addr);
    for (i = 0; i < size; ++i)
        spi_read_write(spi, ((uint8_t *)data)[i]);
    spi_stop(spi);

    w25qxx_wait(spi, WAIT_TIMEOUT);
}

void w25qxx_erase(struct device *spi, uint8_t erase_type, uint32_t addr)
{
    w25qxx_write_enable(spi);
    spi_start(spi);
    spi_read_write(spi, erase_type);
    w25qxx_send_address(spi, addr);
    spi_stop(spi);
    w25qxx_wait(spi, WAIT_TIMEOUT);
}
