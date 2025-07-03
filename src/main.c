/*
 * STM32 main entry point.
 */

#include "stm32.h"
#include "icm42688p.h"
#include "w25qxx.h"

#define DEBUG_LED_GPIO GPIOA
#define DEBUG_LED_PIN  GPIO_Pin_13

#define TEST_EXTI (1 << 0)
#define TEST_PWMI (1 << 1)
#define TEST_ENCODER (1 << 2)
#define TEST_SINGLE_ADC (1 << 3)
#define TEST_DMA (1 << 4)
#define TEST_ADC_DMA (1 << 5)
#define TEST_USART (1 << 6)
#define TEST_I2C_SOFTWARE (1 << 7)
#define TEST_I2C_HARDWARE (1 << 8)
#define TEST_SPI_SOFTWARE (1 << 9)
#define TEST_SPI_HARDWARE (1 << 10)


void exti_irq_handler(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0)
        TRACE("Trigger falling edge on PB9.\n");
}

void usart_receive_handler(void)
{
    /* FIXME: Instead of handling data in irq handler, it's better to push data to a buffer, and set a flag. */
    putchar(USART_ReceiveData(USART1));
}

void test_exti(void)
{
    exti_init(GPIOB, GPIO_Pin_9, exti_irq_handler, EXTI_Trigger_Falling, 1, 1);
}

void test_encoder(void)
{
    encoder_init(TIM3);
    while (1)
    {
        printf("encoder count: %d.\n", (int16_t)encoder_get_count(TIM3));
        delay_ms(20);
    }
}

void test_pwmi(void)
{
    timer_pwm_init(TIM2, TIM_Channel_1, 1000, 100);
    timer_pwm_set_pulse(TIM2, TIM_Channel_1, 16);
    timer_input_capture_init(TIM3, TIM_Channel_1, true);
    while (1)
    {
        TRACE("Frequency: %u, duty %u.\n",
                timer_input_capture_get_frequency(TIM3, TIM_Channel_1),
                timer_input_capture_get_duty(TIM3, TIM_Channel_1));
        delay_ms(20);
    }
}

void test_adc(void)
{
    adc_single_init(ADC1);
    adc_init_channel(ADC1, ADC_Channel_2);
    while (1)
    {
        adc_start_single_convert(ADC1, ADC_Channel_2);
        printf("adc value: %u.\n", adc_wait_value(ADC1));
        delay_ms(20);
    }
}

void test_dma(void)
{
    uint32_t src[4] = {1, 2, 3, 4}, dst[4] = {0};
    unsigned int i;

    dma_m2m_init(DMA1_Channel1, src, dst, sizeof(src[0]));

    while (1)
    {
        for (i = 0; i < ARRAY_SIZE(src); ++i)
            ++src[i];
        printf("src changed.\n");
        printf("src: %lu, %lu, %lu, %lu.\n", src[0], src[1], src[2], src[3]);
        printf("dst: %lu, %lu, %lu, %lu.\n\n", dst[0], dst[1], dst[2], dst[3]);
        delay_ms(1000);

        dma_start_transfer(DMA1_Channel1, ARRAY_SIZE(src));
        dma_wait_transfer(DMA1_Channel1);
        printf("dma done.\n");
        printf("src: %lu, %lu, %lu, %lu.\n", src[0], src[1], src[2], src[3]);
        printf("dst: %lu, %lu, %lu, %lu.\n\n", dst[0], dst[1], dst[2], dst[3]);
        delay_ms(1000);
    }
}

void test_adc_dma(void)
{
    uint16_t adc_value[2];

    adc_dma_init(ADC1, 2, adc_value);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_55Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    while (1)
    {
        printf("adc value: %u, %u.\n", adc_value[0], adc_value[1]);
        delay_ms(20);
    }
}

void test_usart(void)
{
    usart_init(USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, usart_receive_handler);
    while (1)
    {
        usart_send_str(USART1, "Hi!\n");
        delay_ms(1000);
    }
}

void test_software_i2c(void)
{
    struct i2c_device device;

    device.i2c.software.scl.gpio = GPIOB;
    device.i2c.software.scl.pin = GPIO_Pin_10;
    device.i2c.software.sda.gpio = GPIOB;
    device.i2c.software.sda.pin = GPIO_Pin_11;
    icm42688p_init(&device, 0, true);

    while (1)
    {
        int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

        icm42688p_get_accel(&device, &accel_x, &accel_y, &accel_z);
        icm42688p_get_gero(&device, &gyro_x, &gyro_y, &gyro_z);

        printf("accel_x: %d, accel_y: %d, accel_z: %d.\n", accel_x, accel_y, accel_z);
        printf("gyro_x: %d, gyro_y: %d, gyro_z: %d.\n", gyro_x, gyro_y, gyro_z);
        delay_ms(100);
    }
}

void test_hardware_i2c(void)
{
    struct i2c_device device;

    device.i2c.hardware = I2C2;
    icm42688p_init(&device, 0, false);

    while (1)
    {
        int16_t accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;

        icm42688p_get_accel(&device, &accel_x, &accel_y, &accel_z);
        icm42688p_get_gero(&device, &gyro_x, &gyro_y, &gyro_z);

        printf("accel_x: %d, accel_y: %d, accel_z: %d.\n", accel_x, accel_y, accel_z);
        printf("gyro_x: %d, gyro_y: %d, gyro_z: %d.\n", gyro_x, gyro_y, gyro_z);
        delay_ms(100);
    }
}

void test_software_spi(void)
{
    struct spi_software spi;
    uint32_t addr = 0x1000;
    char buffer[32];

    spi.device.type = SPI_SOFTWARE;
    spi.ss.gpio = GPIOA;
    spi.ss.pin = GPIO_Pin_4;
    spi.sck.gpio = GPIOA;
    spi.sck.pin = GPIO_Pin_5;
    spi.miso.gpio = GPIOA;
    spi.miso.pin = GPIO_Pin_6;
    spi.mosi.gpio = GPIOA;
    spi.mosi.pin = GPIO_Pin_7;

    if (!w25qxx_init(&spi.device))
    {
        TRACE("Failed to init w25qxx.\n");
        return;
    }

    w25qxx_erase(&spi.device, W25QXX_INSTR_SECTOR_ERASE_4KB, addr);
    w25qxx_write(&spi.device, addr, "Hello, world!", 14);
    w25qxx_read(&spi.device, addr, buffer, sizeof(buffer));
    TRACE("Read from w25qxx: \"%s\".\n", buffer);

    while (1) {}
}

int main(void)
{
    uint32_t test_case;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    debug_init(USART1, DEBUG_LED_GPIO, DEBUG_LED_PIN);

    test_case = TEST_SPI_SOFTWARE;

    if (test_case & TEST_EXTI)
        test_exti();
    if (test_case & TEST_PWMI)
        test_pwmi();
    if (test_case & TEST_ENCODER)
        test_encoder();
    if (test_case & TEST_SINGLE_ADC)
        test_adc();
    if (test_case & TEST_DMA)
        test_dma();
    if (test_case & TEST_ADC_DMA)
        test_adc_dma();
    if (test_case & TEST_USART)
        test_usart();
    if (test_case & TEST_I2C_SOFTWARE)
        test_software_i2c();
    if (test_case & TEST_I2C_HARDWARE)
        test_hardware_i2c();
    if (test_case & TEST_SPI_SOFTWARE)
        test_software_spi();

    return 0; /* Should never be here. */
}
