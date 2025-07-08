/*
 * RTC & BKP driver functions.
 */

#include "stm32.h"

#define LSE_FREQ 32768

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
