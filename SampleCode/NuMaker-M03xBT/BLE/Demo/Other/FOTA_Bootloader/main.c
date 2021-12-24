/******************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate how to upgrade firmware between USB device and PC through USB DFU( Device Firmware Upgrade) class.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "porting_ota.h"
#include "porting_flash.h"

#define V6M_AIRCR_VECTKEY_DATA    0x05FA0000UL
#define V6M_AIRCR_SYSRESETREQ     0x00000004UL

extern uint32_t flash_fota_info_addr;       // The start address of FOTA info partition

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    /* Enable Internal High speed RC oscillator (HIRC) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;
    /* Waiting for Internal High speed RC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC ;

    SystemCoreClockUpdate();
    /* lock protected registers */
    SYS_LockReg();
}


static int  load_image_into_flash(uint32_t image_base, uint32_t flash_addr, uint32_t image_size)
{
    uint32_t i, j, u32Data, u32RemainSize;

    for (i = 0; i < image_size; i += FLASH_PAGE_SIZE)
    {

        FMC_Erase(flash_addr + i);//page erase
        for (j = 0; j < FLASH_PAGE_SIZE; j += FLASH_PROGRAM_SIZE) //write 4 bytes
        {
            u32Data = FMC_Read(image_base + i + j);
            FMC_Write(flash_addr + i + j, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
            /* Invalidate cache after FMC_Write() to fix ISP issue */
            FMC->FTCTL |= BIT9;
            while ((FMC->FTCTL & BIT9) == BIT9);
#endif
        }
    }

    /*if image size not align with page size*/
    if (image_size != i)
    {
        u32RemainSize = i - image_size;

        FMC_Erase(flash_addr + i);//page erase
        for (j = 0; j < u32RemainSize; j += FLASH_PROGRAM_SIZE) //write 4 bytes
        {
            u32Data = FMC_Read(image_base + i + j);
            FMC_Write(flash_addr + i + j, u32Data);
#if (_CHIP_SELECTION_ == _CHIP_M032BT)
            FMC->FTCTL |= BIT9;
            while ((FMC->FTCTL & BIT9) == BIT9);
#endif
        }

    }

    return 0;
}

uint32_t crc32(uint32_t flash_addr, uint32_t data_len)
{
    uint8_t RemainLen = (data_len & 0x03);
    uint32_t i, j, k;
    uint32_t ChkSum = ~0;
    uint32_t Len = (data_len >> 2), Index = 0, Read;

    for (i = 0; i < Len; i ++)
    {
        //get 32 bits at one time
        Read = FMC_Read(flash_addr + Index);
        Index += 4;

        //get the CRC of 32 bits
        for (j = 0; j < 32; j += 8)
        {
            //get the CRC of 8 bits
            ChkSum ^= ((Read >> j) & 0xFF);
            for (k = 0; k < 8; k ++)
            {
                ChkSum = (ChkSum & 1) ? (ChkSum >> 1) ^ 0xedb88320 : ChkSum >> 1;
            }
        }
    }
    /*if data_len not align with flash programming size*/
    if (RemainLen > 0)
    {
        //get 32 bits at one time
        Read = FMC_Read(flash_addr + Index);
        Index += 4;

        //get the CRC of 32 bits
        for (j = 0; j < (RemainLen << 3); j += 8)
        {
            //get the CRC of 8 bits
            ChkSum ^= ((Read >> j) & 0xFF);
            for (k = 0; k < 8; k ++)
            {
                ChkSum = (ChkSum & 1) ? (ChkSum >> 1) ^ 0xedb88320 : ChkSum >> 1;
            }
        }
    }
    ChkSum = ~ChkSum;
    return ChkSum;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t Bank1Ready;
    uint32_t Bank1Crc;
    uint32_t Bank1DataLen;
    uint32_t Bank1StartAddress;
    uint32_t Bank0StartAddress;
    uint32_t PageIdx = 0;

    /* Init system and multi-function I/O */
    SYS_Init();

    /* Unlock write-protected registers */
    SYS_UnlockReg();
    FMC_Open();

    if (!flash_fota_info_addr)
    {
        /* Set the base address of flash partitions */
        setBLE_FlashPartitionsBA(0);
    }

    Bank1Ready = FMC_Read(flash_fota_info_addr + sizeof(uint32_t));

    if (Bank1Ready == FOTA_IMAGE_READY)/*check firmware upgrade need to start*/
    {
        Bank1Crc = FMC_Read(flash_fota_info_addr + (sizeof(uint32_t) * 2));
        Bank1DataLen = FMC_Read(flash_fota_info_addr + (sizeof(uint32_t) * 3));
        Bank1StartAddress = FMC_Read(flash_fota_info_addr + (sizeof(uint32_t) * 4));
        Bank0StartAddress = FMC_Read(flash_fota_info_addr + (sizeof(uint32_t) * 5));

        FMC_ENABLE_AP_UPDATE();
        //new firmware validation
        if (Bank1Crc == crc32((uint32_t)Bank1StartAddress, Bank1DataLen))
        {
            //new firmware replacement
            load_image_into_flash(Bank1StartAddress, Bank0StartAddress, Bank1DataLen );
        }

        for (PageIdx = 0 ; PageIdx < Bank1DataLen ; PageIdx += FLASH_PAGE_SIZE)
        {
            FMC_Erase((uint32_t)Bank1StartAddress + PageIdx) ;
        }
        //reset firmware upgrade flag
        FMC_Erase(flash_fota_info_addr);

        FMC_DISABLE_AP_UPDATE() ;

    }
    /* Disable FMC ISP function */
    FMC_Close();

    //printf("APROM start\n");
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);//boot selection, FMC_SET_APROM_BOOT
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);//reset

}

