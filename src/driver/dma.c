/*
 * DMA driver functions.
 */

#include "stm32.h"

static uint32_t dma_channel_num(DMA_Channel_TypeDef *channel)
{
    return channel - DMA1_Channel1 + 1;
}

bool dma_init(DMA_Channel_TypeDef *channel,
        void *src_addr, void *dst_addr, uint32_t data_size, bool m2m, bool periph_inc, bool circular)
{
    DMA_InitTypeDef dma_init_type_def;

    rcc_enable(BUS_AHB, RCC_AHBPeriph_DMA1);

    DMA_StructInit(&dma_init_type_def);
    dma_init_type_def.DMA_PeripheralBaseAddr = (uint32_t)src_addr;
    dma_init_type_def.DMA_PeripheralInc = periph_inc ? DMA_PeripheralInc_Enable: DMA_PeripheralInc_Disable;
    dma_init_type_def.DMA_MemoryBaseAddr = (uint32_t)dst_addr;
    dma_init_type_def.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init_type_def.DMA_M2M = m2m ? DMA_M2M_Enable : DMA_M2M_Disable;
    dma_init_type_def.DMA_Mode = circular ? DMA_Mode_Circular : DMA_Mode_Normal;
    dma_init_type_def.DMA_BufferSize = 1;
    switch (data_size)
    {
        case sizeof(uint8_t):
            dma_init_type_def.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            dma_init_type_def.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            break;
        case sizeof(uint16_t):
            dma_init_type_def.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
            dma_init_type_def.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
            break;
        case sizeof(uint32_t):
            dma_init_type_def.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
            dma_init_type_def.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
            break;
        default:
            TRACE("Invalid data size: %u.\n", data_size);
            return false;
    }
    DMA_Init(channel, &dma_init_type_def);

    TRACE("Inited DMA1 channel %u, src %p, dst %p, size %u.\n",
            dma_channel_num(channel), src_addr, dst_addr, data_size);

    return true;
}

bool dma_m2m_init(DMA_Channel_TypeDef *channel,
        void *src_addr, void *dst_addr, uint32_t data_size)
{
    return dma_init(channel, src_addr, dst_addr, data_size, true, true, false);
}

void dma_start_transfer(DMA_Channel_TypeDef *channel, uint16_t data_count)
{
    DMA_Cmd(channel, DISABLE);
    DMA_SetCurrDataCounter(channel, data_count);
    DMA_Cmd(channel, ENABLE);
}

void dma_wait_transfer(DMA_Channel_TypeDef *channel)
{
    uint32_t tc_flag = DMA1_FLAG_TC1 << (4 * (dma_channel_num(channel) - 1));
    while (DMA_GetFlagStatus(tc_flag) != SET) {}
    DMA_ClearFlag(tc_flag);
}
