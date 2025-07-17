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
#define TEST_BKP (1 << 11)
#define TEST_RTC (1 << 12)
#define TEST_CLOCK_FREQ (1 << 13)
#define TEST_SLEEP_MODE (1 << 14)
#define TEST_STOP_MODE (1 << 15)
#define TEST_STANDBY_MODE (1 << 16)


static char usart1_recv_buffer[64];
static uint8_t usart1_recv_size = 0;
static bool exti_triggered = 0;

void usart1_recv_handler(void)
{
    usart1_recv_buffer[usart1_recv_size++] = USART_ReceiveData(USART1);
}

void exti_irq_handler(void)
{
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
        exti_triggered = true;
}

static void test_exti(void)
{
    exti_init(GPIOA, GPIO_Pin_5, exti_irq_handler, EXTI_Trigger_Falling, 1, 1);

    while (1)
    {
        printf("Hi!\n");

        if (exti_triggered)
        {
            printf("Trigger EXTI.\n");
            exti_triggered = false;
        }

        delay_ms(500);
    }
}

static void test_encoder(void)
{
    encoder_init(TIM3);
    while (1)
    {
        printf("encoder count: %d.\n", (int16_t)encoder_get_count(TIM3));
        delay_ms(20);
    }
}

static void test_pwmi(void)
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

static void test_adc(void)
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

static void test_dma(void)
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

static void test_adc_dma(void)
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

static void test_usart(void)
{
    usart_init(USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, usart1_recv_handler);
    memset(usart1_recv_buffer, 0, sizeof(usart1_recv_buffer));

    while (1)
    {
        usart_send_str(USART1, "Hi!\n");

        if (usart1_recv_size != 0 && usart1_recv_buffer[usart1_recv_size - 1] == '\n')
        {
            printf("Received: %s", usart1_recv_buffer);
            memset(usart1_recv_buffer, 0, sizeof(usart1_recv_buffer));
            usart1_recv_size = 0;
        };

        delay_ms(1000);
    }
}

static void test_software_i2c(void)
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

static void test_hardware_i2c(void)
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

static void test_spi_software(void)
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

static void test_spi_hardware(void)
{
    const char *data = "Im your father.";
    struct spi_hardware spi;
    uint32_t addr = 0x1000;
    char buffer[32];

    spi.device.type = SPI_HARDWARE;
    spi.hardware = SPI1;

    if (!w25qxx_init(&spi.device))
    {
        TRACE("Failed to init w25qxx.\n");
        return;
    }

    w25qxx_erase(&spi.device, W25QXX_INSTR_SECTOR_ERASE_4KB, addr);
    w25qxx_write(&spi.device, addr, data, strlen(data) + 1);
    w25qxx_read(&spi.device, addr, buffer, sizeof(buffer));
    TRACE("Read from w25qxx: \"%s\".\n", buffer);

    while (1) {}
}

static void test_bkp(void)
{
    uint16_t data;
    bkp_init();
    //bkp_write(BKP_DR1, 0x1234);
    data = BKP_ReadBackupRegister(BKP_DR1);
    printf("BKP_DR1: %#x.\n", data);
    while (1) {}
}

static void test_rtc(void)
{
    rtc_init(0);
    while (1)
    {
        printf("RTC counter: %lu.\n", RTC_GetCounter());
        delay_ms(200);
    }
}

static void test_clock_freq(void)
{
    /* Change SYSCLK_FREQ_XXX defines in system_stm32f10x.c to change clock frequency. */
    printf("System clock frequency: %lu.\n", SystemCoreClock);
    while (1)
    {
        delay_ms(1000);
        printf("Bling!\n");
    }
}

static void test_sleep_mode(void)
{
    usart_init(USART1, 9600, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, usart1_recv_handler);
    memset(usart1_recv_buffer, 0, sizeof(usart1_recv_buffer));

    while (1)
    {
        pwr_sleep(PWR_WFI);

        usart_send_str(USART1, "Hi!\n");

        if (usart1_recv_size != 0 && usart1_recv_buffer[usart1_recv_size - 1] == '\n')
        {
            printf("Received: %s", usart1_recv_buffer);
            memset(usart1_recv_buffer, 0, sizeof(usart1_recv_buffer));
            usart1_recv_size = 0;
        };
    }
}

static void test_stop_mode(void)
{
    exti_init(GPIOA, GPIO_Pin_5, exti_irq_handler, EXTI_Trigger_Falling, 1, 1);

    /* Enable the PWR clock before calling PWR functions. */
    rcc_enable(BUS_APB1, RCC_APB1Periph_PWR);

    while (1)
    {
        /* Delay to make sure USART has sent all chars before entering stop mode. */
        delay_ms(100);

        pwr_stop(false, PWR_WFI);

        printf("Hi!\n");

        if (exti_triggered)
        {
            printf("Trigger EXTI.\n");
            exti_triggered = false;
        }
    }
}

void test_standby_mode(void)
{
    uint16_t rtc_inited_magic = 0xdead;
    uint32_t alarm;

    bkp_init();
    if (BKP_ReadBackupRegister(BKP_DR1) != rtc_inited_magic)
    {
        rtc_init(0);
        BKP_WriteBackupRegister(BKP_DR1, rtc_inited_magic);
    }
    else
    {
        rtc_simple_init();
    }

    alarm = RTC_GetCounter() + 3;
    RTC_SetAlarm(alarm);

    while (1)
    {
        delay_ms(500);
        printf("RTC counter: %lu, alarm: %lu, RTC_FLAG_ALR %d.\n",
                RTC_GetCounter(), alarm, RTC_GetFlagStatus(RTC_FLAG_ALR));
        pwr_standby();
    }
}

int main(void)
{
    uint32_t test_case;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    debug_init(USART1, DEBUG_LED_GPIO, DEBUG_LED_PIN);

    test_case = TEST_STANDBY_MODE;

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
        test_spi_software();
    if (test_case & TEST_SPI_HARDWARE)
        test_spi_hardware();
    if (test_case & TEST_BKP)
        test_bkp();
    if (test_case & TEST_RTC)
        test_rtc();
    if (test_case & TEST_CLOCK_FREQ)
        test_clock_freq();
    if (test_case & TEST_SLEEP_MODE)
        test_sleep_mode();
    if (test_case & TEST_STOP_MODE)
        test_stop_mode();
    if (test_case & TEST_STANDBY_MODE)
        test_standby_mode();

    return 0; /* Should never be here. */
}
