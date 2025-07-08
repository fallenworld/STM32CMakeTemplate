/*
 * SPI driver functions.
 */

#include "stm32.h"

#define WAIT_TIMEOUT 10000

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

static struct spi_hardware_info
{
    SPI_TypeDef *spi;
    uint32_t bus;
    uint32_t periph;
    struct gpio_pin nss, sck, miso, mosi;
} spi_hardware_list[] =
{
    {SPI1, BUS_APB2, RCC_APB2Periph_SPI1, {GPIOA, GPIO_Pin_4},  {GPIOA, GPIO_Pin_5},  {GPIOA, GPIO_Pin_6},  {GPIOA, GPIO_Pin_7}},
    {SPI2, BUS_APB1, RCC_APB1Periph_SPI2, {GPIOB, GPIO_Pin_12}, {GPIOB, GPIO_Pin_13}, {GPIOB, GPIO_Pin_14}, {GPIOB, GPIO_Pin_15}},
};

static const struct spi_hardware_info *spi_hardware_info_find(const SPI_TypeDef *spi)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(spi_hardware_list); ++i)
    {
        if (spi_hardware_list[i].spi == spi)
            return &spi_hardware_list[i];
    }
    return NULL;
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
    const struct spi_hardware_info *info = spi_hardware_info_find(spi->hardware);
    SPI_InitTypeDef init_def;

    if (!info)
    {
        TRACE("Invalid hardware SPI %p.\n", spi);
        return false;
    }

    if (!gpio_pin_init(&info->nss, GPIO_Mode_Out_PP)
            || !gpio_pin_init(&info->sck, GPIO_Mode_AF_PP)
            || !gpio_pin_init(&info->miso, GPIO_Mode_IPU)
            || !gpio_pin_init(&info->mosi, GPIO_Mode_AF_PP))
        return false;

    rcc_enable(info->bus, info->periph);

    SPI_StructInit(&init_def);
    init_def.SPI_Mode = SPI_Mode_Master;
    init_def.SPI_NSS = SPI_NSS_Soft;
    init_def.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    SPI_Init(spi->hardware, &init_def);

    SPI_Cmd(spi->hardware, ENABLE);
    gpio_pin_write(&info->nss, 1);
    TRACE("Inited hardware SPI %p.\n", spi);

    return true;
}

static void spi_hardware_start(const struct spi_hardware *spi)
{
    const struct spi_hardware_info *info = spi_hardware_info_find(spi->hardware);
    if (info)
        gpio_pin_write(&info->nss, 0);
    else
        TRACE("Invalid hardware SPI %p.\n", spi);
}

static void spi_hardware_stop(const struct spi_hardware *spi)
{
    const struct spi_hardware_info *info = spi_hardware_info_find(spi->hardware);
    if (info)
        gpio_pin_write(&info->nss, 1);
    else
        TRACE("Invalid hardware SPI %p.\n", spi);
}

static uint8_t spi_hardware_read_write(const struct spi_hardware *spi, uint8_t data)
{
    uint32_t timeout = WAIT_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(spi->hardware, SPI_I2S_FLAG_TXE) != SET)
    {
        if (!timeout--)
        {
            TRACE("Timeout waiting for SPI TXE.\n");
            return 0;
        }
    }

    SPI_I2S_SendData(spi->hardware, data);

    while (SPI_I2S_GetFlagStatus(spi->hardware, SPI_I2S_FLAG_RXNE) != SET)
    {
        if (!timeout--)
        {
            TRACE("Timeout waiting for SPI RXNE.\n");
            return 0;
        }
    }

    return SPI_I2S_ReceiveData(spi->hardware);
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
