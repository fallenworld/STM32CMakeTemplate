/*
 * PWR & BKP & RTC driver functions.
 */

#include "stm32.h"

#define LSE_FREQ 32768

void pwr_sleep(uint32_t mode)
{
    rcc_enable(BUS_APB1, RCC_APB1Periph_PWR);

    if (mode == PWR_WFI)
        __WFI();
    else if (mode == PWR_WFE)
        __WFE();
    else
        TRACE("Invalid mode %u.\n", mode);
}

void pwr_stop(bool low_power, uint32_t mode)
{
    if (mode != PWR_WFI && mode != PWR_WFE)
        TRACE("Invalid mode %u.\n", mode);

    rcc_enable(BUS_APB1, RCC_APB1Periph_PWR);

    PWR_EnterSTOPMode(low_power ? PWR_Regulator_LowPower : PWR_Regulator_ON, mode);
    /* System clock will be set to HSI resuming from stop mode.
     * So we need to call SystemInit() here to reset it to HSE. */
    SystemInit();
}

void pwr_standby(void)
{
    rcc_enable(BUS_APB1, RCC_APB1Periph_PWR);
    PWR_ClearFlag(PWR_FLAG_WU);
    PWR_EnterSTANDBYMode();
}

void bkp_init(void)
{
    rcc_enable(BUS_APB1, RCC_APB1Periph_PWR);
    rcc_enable(BUS_APB1, RCC_APB1Periph_BKP);
    PWR_BackupAccessCmd(ENABLE);
}

void rtc_init(uint32_t initial_count)
{
    uint32_t timeout = 10000;
    bkp_init();

    RCC_LSEConfig(RCC_LSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) != SET)
    {
        if (!timeout--)
        {
            TRACE("Timeout waiting for LSE.\n");
            return;
        }
    }

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    RCC_RTCCLKCmd(ENABLE);

    RTC_WaitForSynchro();
    RTC_WaitForLastTask();

    RTC_SetPrescaler(LSE_FREQ - 1);
    RTC_WaitForLastTask();

    RTC_SetCounter(initial_count);
    RTC_WaitForLastTask();
}

void rtc_simple_init(void)
{
    bkp_init();
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
}
