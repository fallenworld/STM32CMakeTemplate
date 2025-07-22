/*
 * FLASH driver functions.
 */

#include "stm32.h"

uint16_t flash_get_size(void)
{
    return *(__IO uint16_t *)0x1FFFF7E0;
}

void flash_get_device_id(uint32_t *id_low, uint32_t *id_mid, uint32_t *id_high)
{
    *id_low  = *(__IO uint32_t *)0x1FFFF7E8;
    *id_mid  = *(__IO uint32_t *)0x1FFFF7EC;
    *id_high = *(__IO uint32_t *)0x1FFFF7F0;
}

void flash_write_16(uint32_t addr, uint16_t data)
{
    FLASH_Unlock();
    FLASH_ProgramHalfWord(addr, data);
    FLASH_Lock();
}

void flash_write_32(uint32_t addr, uint32_t data)
{
    FLASH_Unlock();
    FLASH_ProgramWord(addr, data);
    FLASH_Lock();
}

void flash_erase_page(uint32_t addr)
{
    FLASH_Unlock();
    FLASH_ErasePage(addr);
    FLASH_Lock();
}

void flash_erase_all(void)
{
    /* It will also erase the code in flash!! */
    FLASH_Unlock();
    FLASH_EraseAllPages();
    FLASH_Lock();
}