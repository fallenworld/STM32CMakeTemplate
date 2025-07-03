/*
 * STM32 SPI driver functions.
 */

#include "stm32.h"

#define DEF_SPI_OP(op, spi, ...)                                                 \
    switch (spi->type)                                                           \
    {                                                                            \
        case SPI_SOFTWARE:                                                       \
            return spi_software_##op((struct spi_software *)spi, ##__VA_ARGS__); \
        case SPI_HARDWARE:                                                       \
            return spi_hardware_##op((struct spi_hardware *)spi, ##__VA_ARGS__); \
        default:                                                                 \
            TRACE("Invalid type %#x.\n", spi->type);                             \
    }

static bool spi_software_init(const struct spi_software *spi)
{
    if (!gpio_pin_init(&spi->ss, GPIO_Mode_Out_PP)
            || !gpio_pin_init(&spi->sck, GPIO_Mode_Out_PP)
            || !gpio_pin_init(&spi->miso, GPIO_Mode_IPU)
            || !gpio_pin_init(&spi->mosi, GPIO_Mode_Out_PP))
        return false;

    gpio_pin_write(&spi->ss, 1);
    gpio_pin_write(&spi->sck, 0);

    TRACE("Inited software SPI.\n");

    return true;
}

static void spi_software_start(const struct spi_software *spi)
{
    gpio_pin_write(&spi->ss, 0);
}

static void spi_software_stop(const struct spi_software *spi)
{
    gpio_pin_write(&spi->ss, 1);
}

static uint8_t spi_software_read_write(const struct spi_software *spi, uint8_t data)
{
    uint8_t i, read = 0;

    for (i = 0; i < 8; ++i)
    {
        gpio_pin_write(&spi->mosi, !!(data & (0x80 >> i)));
        gpio_pin_write(&spi->sck, 1);
        if (gpio_pin_read(&spi->miso))
            read |= (0x80 >> i);
        gpio_pin_write(&spi->sck, 0);
    }

    return read;
}

static bool spi_hardware_init(const struct spi_hardware *spi)
{
    (void)spi;
    return false;
}

static void spi_hardware_start(const struct spi_hardware *spi)
{
    (void)spi;
}

static void spi_hardware_stop(const struct spi_hardware *spi)
{
    (void)spi;
}

static uint8_t spi_hardware_read_write(const struct spi_hardware *spi, uint8_t data)
{
    (void)spi;
    (void)data;
    return 0;
}

bool spi_init(struct device *spi)
{
    DEF_SPI_OP(init, spi);
    return false;
}

void spi_start(struct device *spi)
{
    DEF_SPI_OP(start, spi);
}

void spi_stop(struct device *spi)
{
    DEF_SPI_OP(stop, spi);
}

uint8_t spi_read_write(struct device *spi, uint8_t data)
{
    DEF_SPI_OP(read_write, spi, data);
    return 0;
}
