/*----------------------------------------------------------------------------*/
/* This file implement MCU peripherals like: SPI for BLE                      */
/*----------------------------------------------------------------------------*/
#include "rf_phy.h"
#include "porting_spi.h"

/******************************************************************************
 * Variables
 ******************************************************************************/


/******************************************************************************
 * Public Functions
 ******************************************************************************/

/** This function is used to initial MCU SPI.
 *
 * @return none
 */
void setBLE_SpiInit(void)
{
    /* Select PCLK1 as the clock source of SPI0*/
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);
    /* Enable PDMA clock source */
    CLK_EnableModuleClock(PDMA_MODULE);

#if (_CHIP_SELECTION_==_CHIP_M031BT)
    /* MCU SPI pin initialization */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & ~(SYS_GPD_MFPL_PD3MFP_Msk |
                                       SYS_GPD_MFPL_PD2MFP_Msk |
                                       SYS_GPD_MFPL_PD1MFP_Msk |
                                       SYS_GPD_MFPL_PD0MFP_Msk)) |
                    (SYS_GPD_MFPL_PD3MFP_SPI0_SS   |
                     SYS_GPD_MFPL_PD2MFP_SPI0_CLK  |
                     SYS_GPD_MFPL_PD1MFP_SPI0_MISO |
                     SYS_GPD_MFPL_PD0MFP_SPI0_MOSI);
#elif (_CHIP_SELECTION_==_CHIP_M032BT)
    /* MCU SPI pin initialization */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & ~(SYS_GPF_MFPL_PF7MFP_Msk |
                                       SYS_GPF_MFPL_PF6MFP_Msk)) |
                    (SYS_GPF_MFPL_PF7MFP_SPI0_MISO |
                     SYS_GPF_MFPL_PF6MFP_SPI0_MOSI);
    SYS->GPF_MFPH = (SYS->GPF_MFPH & ~(SYS_GPF_MFPH_PF9MFP_Msk |
                                       SYS_GPF_MFPH_PF8MFP_Msk)) |
                    (SYS_GPF_MFPH_PF9MFP_SPI0_SS   |
                     SYS_GPF_MFPH_PF8MFP_SPI0_CLK);
#endif

    /* SPI master, clk=8M, mode 0, 8-bit, MSB first */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, SPI_CLK_FREQ);

#if (SPI_SS_CONTROL == AUTO_SPI_SS)
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active */
    SPI_EnableAutoSS(SPI0, SPI_SS, SPI_SS_ACTIVE_LOW);
#else
    /* Disable the automatic hardware slave select function.*/
    SPI_DisableAutoSS(SPI0);
    setBLE_SpiSSHigh();
#endif //#if (SPI_SS_CONTROL == AUTO_SPI_SS)
}



#if defined (__CC_ARM)
#pragma push
#pragma Otime
#endif


/** This function is used to read RF 1 byte register.
 *
 * @param[in] regAddr : register address
 * @return 1 byte RX data.
 */
uint8_t setBLE_SpiOneByteRx(uint8_t regAddr)
{
    uint32_t u32i;

    while (1)
    {
        InterruptDisable();
        if (SPI_IS_BUSY(SPI0) == 0)   //if (SPI is free)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            SPI0_ClearRxFIFO();
#else
            setBLE_SpiSSHigh();
            SPI0_ClearRxFIFO();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            SPI_WRITE_TX(SPI0, (regAddr | 0x80));   //write 1st byte: (regAddr & 0x7F) | 0x80
            SPI_DISABLE_RX_PDMA(SPI0);
            if ((regAddr & 0x80))
            {
                SPI_WRITE_TX(SPI0, 0x01);   //write 2nd byte: (regAddr & 0x80)>>7
            }
            else
            {
                SPI_WRITE_TX(SPI0, 0x00);
            }
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)  //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);  //Clear the PDMA transfer done flags
                }
            }
            break;
        }
        InterruptEnable();
    }

    SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte
    SPI_WRITE_TX(SPI0, 0xF5);   //write 1 dummy byte to read 1 byte data

    while (SPI_GET_RX_FIFO_COUNT(SPI0) != 4);
    SPI_READ_RX(SPI0);
    SPI_READ_RX(SPI0);
    SPI_READ_RX(SPI0);
    u32i = SPI_READ_RX(SPI0);

    SPI0_ClearRxFIFO();
    InterruptEnable();
    return (uint8_t)u32i;
}


/** This function is used to read RF 1 byte register or RX_FIFO in ISR.
 *
 * @param[in] regAddr : register address
 * @return 1 byte RX data.
 */
uint8_t setBLE_SpiOneByteRxIsr(uint8_t regAddr)
{
    uint32_t u32i;

    while (1)
    {
        if (SPI_IS_BUSY(SPI0) == 0)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            SPI0_ClearRxFIFO();
#else
            setBLE_SpiSSHigh();
            SPI0_ClearRxFIFO();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            SPI_WRITE_TX(SPI0, (regAddr | 0x80));   //write 1st byte: (regAddr & 0x7F) | 0x80
            SPI_DISABLE_RX_PDMA(SPI0);
            if ((regAddr & 0x80))
            {
                SPI_WRITE_TX(SPI0, 0x01);   //write 2nd byte: (regAddr & 0x80)>>7
            }
            else
            {
                SPI_WRITE_TX(SPI0, 0x00);
            }
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)   //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
                }
            }
            break;
        }
    }

    if (regAddr == RX_BUFFER_READ_PORT) //read RX_FIFO
    {
        SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte for MP read RX_FIFO
        SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte for MP read RX_FIFO

        SPI_WRITE_TX(SPI0, 0xF5);   //write 1 dummy byte to read 1 byte data

        while (SPI_GET_RX_FIFO_COUNT(SPI0) != 5);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
    }
    else            //read register
    {
        SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte for MP read register

        SPI_WRITE_TX(SPI0, 0xF5);   //write 1 dummy byte to read 1 byte data

        while (SPI_GET_RX_FIFO_COUNT(SPI0) != 4);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
    }

    u32i = SPI_READ_RX(SPI0);
    SPI0_ClearRxFIFO();

    return (uint8_t)u32i;
}


/** This function is used to write RF 1 byte register.
 *
 * @note Not use for writing TX buffer.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @return none
 */
void setBLE_SpiOneByteTx(uint8_t regAddr, uint8_t u8SrcData)
{
    uint32_t u32i;

    while (1)
    {
        InterruptDisable();
        if (SPI_IS_BUSY(SPI0) == 0)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            __NOP();
#else
            setBLE_SpiSSHigh();
            __NOP();
            __NOP();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)   //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
                }
            }
            break;
        }
        InterruptEnable();
    }
    SPI_WRITE_TX(SPI0, (regAddr & 0x7F)); //write 1st byte: regAddr & 0x7F

    if ((regAddr & 0x80))
    {
        SPI_WRITE_TX(SPI0, 0x01);   //write 2nd byte: (regAddr & 0x80) >> 7
    }
    else
    {
        SPI_WRITE_TX(SPI0, 0x00);
    }

    SPI_WRITE_TX(SPI0, u8SrcData);

    InterruptEnable();
}


/** This function is used to write RF 1 byte register in ISR.
 *
 * @note Not use for writing TX buffer.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @return none
 */
void setBLE_SpiOneByteTxIsr(uint8_t regAddr, uint8_t u8SrcData)
{
    uint32_t u32i;
    while (1)
    {
        if (SPI_IS_BUSY(SPI0) == 0)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            __NOP();
#else
            setBLE_SpiSSHigh();
            __NOP();
            __NOP();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)   //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
                }
            }
            break;
        }
    }
    SPI_WRITE_TX(SPI0, (regAddr & 0x7F));   //write 1st byte: regAddr & 0x7F
    if ((regAddr & 0x80))
    {
        SPI_WRITE_TX(SPI0, 0x01);   //write 2nd byte: (regAddr & 0x80) >> 7
    }
    else
    {
        SPI_WRITE_TX(SPI0, 0x00);
    }

    SPI_WRITE_TX(SPI0, u8SrcData);

}


/** This function is used to write RF 2 byte register in ISR.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @return none
 */
void setBLE_SpiTwoByteTxIsr(uint8_t regAddr, uint8_t *u8SrcAddr)
{
    uint32_t u32i;

    while (1)
    {
        if (SPI_IS_BUSY(SPI0) == 0)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            __NOP();
#else
            setBLE_SpiSSHigh();
            __NOP();
            __NOP();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)   //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
                }
            }
            break;
        }
    }
    SPI_WRITE_TX(SPI0, (regAddr & 0x7F));   //write 1st byte: regAddr & 0x7F

    if ((regAddr & 0x80))
    {
        SPI_WRITE_TX(SPI0, 0x01);   //write 2nd byte: (regAddr & 0x80) >> 7
    }
    else
    {
        SPI_WRITE_TX(SPI0, 0x00);
    }

    if (regAddr == TX_BUFFER_WRITE_PORT)    //if (writing to TX_buffer)
    {
        uint32_t ram_start_addr;

        ram_start_addr = getBLE_TxFIFOAddr();
        SPI_WRITE_TX(SPI0, (ram_start_addr & 0xFF));          //write 3rd byte: ram_start_addr & 0xFF
        SPI_WRITE_TX(SPI0, (ram_start_addr & 0x0100) >> 8);   //write 4th byte: (ram_start_addr & 0x0100)>>8 | (b7<<7),  b7=1/0 means fill payload/header
    }
    SPI_WRITE_TX(SPI0, *(u8SrcAddr));
    SPI_WRITE_TX(SPI0, *(u8SrcAddr + 1));
}

/*------------- SPI_PDMA ---------------*/


/** This function is used to initial MCU SPI PDMA.
 *
 * @return none
 */
void setBLE_SpiPDMAInit(void)
{
    //Reset PDMA module
    SYS_ResetModule(PDMA_RST);

    //Enable PDMA channels
    PDMA_Open(PDMA, SPI_OPENED_CH);

    //Single request type. SPI only support PDMA single request type.
    PDMA_SetBurstType(PDMA, SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA, SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);

    //Set source/destination attributes
    PDMA_SetTransferAddr(PDMA, SPI_MASTER_TX_DMA_CH, (uint32_t)NULL, PDMA_SAR_INC, (uint32_t)&SPI0->TX, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA, SPI_MASTER_RX_DMA_CH, (uint32_t)&SPI0->RX, PDMA_SAR_FIX, (uint32_t)NULL, PDMA_DAR_INC);

    //Set request source; set basic mode.
    PDMA_SetTransferMode(PDMA, SPI_MASTER_TX_DMA_CH, PDMA_SPI0_TX, FALSE, 0);
    PDMA_SetTransferMode(PDMA, SPI_MASTER_RX_DMA_CH, PDMA_SPI0_RX, FALSE, 0);

    //Disable table interrupt
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
}


/** This function is used to wait PDMA complete the operation.
 *
 * @return  SPI PDMA Result success or not.
 * @retval  0:  Success
 * @retval  1:  FAIL
 */
uint32_t setBLE_SpiPDMAWaitFinish(void)
{
    uint32_t u32i;

    while (SPI_IS_BUSY(SPI0));
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
    __NOP();
#else
    setBLE_SpiSSHigh();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
    u32i = PDMA_GET_INT_STATUS(PDMA);   //Get interrupt status
    if ((u32i & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
    {
        if (u32i & PDMA_INTSTS_TDIF_Msk)    //Check the PDMA transfer done interrupt flag
        {
            u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
            if (u32i)   //Check the PDMA transfer done flags
            {
                PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
            }
            u32i = 0;
        }
        else if (u32i & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))    //Check the DMA time-out interrupt flag
        {
            PDMA->INTSTS = u32i & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk);    //Clear the time-out flag
            u32i = 1;
        }
        else    //Check the DMA transfer abort interrupt flag, (u32i & PDMA_INTSTS_ABTIF_Msk)
        {
            u32i = PDMA_GET_ABORT_STS(PDMA);    //Get the target abort flag
            PDMA_CLR_ABORT_FLAG(PDMA, u32i);    //Clear the target abort flag
            u32i = 1;
        }
    }
    else
    {
        u32i = 0;
    }
    return u32i;
}


/** This function is used to using SPI PDMA read RF in ISR.
 *
 * @attention Because of the set-and-forget SPI DMA behavior, user should avoid modification of the source data.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @param[in] u32TransCount : transfer count
 * @return none
 */
void setBLE_SpiPDMARxIsr(uint8_t regAddr, uint32_t u32DstAddr, uint32_t u32TransCount)
{
    uint32_t u32i;

    while (1)
    {
        if (SPI_IS_BUSY(SPI0) == 0)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            __NOP();
#else
            setBLE_SpiSSHigh();
            __NOP();
            __NOP();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)   //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
                }
            }
            break;
        }
    }
    SPI0_ClearRxFIFO();
    SPI_WRITE_TX(SPI0, ((regAddr & 0x7F) | 0x80));    //1st byte (regAddr & 0x7F) | 0x80
    if ((regAddr & 0x80))
    {
        SPI_WRITE_TX(SPI0, 0x01);   //2nd byte (regAddr & 0x80)>>7
    }
    else
    {
        SPI_WRITE_TX(SPI0, 0x00);
    }

    if (regAddr == RX_BUFFER_READ_PORT) //read RX_FIFO
    {
        SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte for MP read RX_FIFO
        SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte for MP read RX_FIFO
    }
    else    //read register
    {
        SPI_WRITE_TX(SPI0, 0xFF);   //1 more dummy byte for MP read register
    }

    SPI_DISABLE_RX_PDMA(SPI0);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].SA = 0; //In SPI read, Master write dummy data to generate SPI clock

    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk | PDMA_DSCT_CTL_OPMODE_Msk);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= ((PDMA_WIDTH_8 | PDMA_OP_BASIC) | ((u32TransCount - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos));
    SPI_TRIGGER_TX_PDMA(SPI0);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].DA = u32DstAddr;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk | PDMA_DSCT_CTL_OPMODE_Msk);
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= ((PDMA_WIDTH_8 | PDMA_OP_BASIC) | ((u32TransCount - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos));

    if (regAddr == RX_BUFFER_READ_PORT) //read RX_FIFO
    {
        while (SPI_GET_RX_FIFO_COUNT(SPI0) != 4);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
    }
    else   //read register
    {
        while (SPI_GET_RX_FIFO_COUNT(SPI0) != 3);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
        SPI_READ_RX(SPI0);
    }

    SPI_TRIGGER_RX_PDMA(SPI0);
}


/** This function is used to using SPI PDMA write RF in ISR.
 *
 * @attention Because of the set-and-forget SPI DMA behavior, user should avoid modification of the source data.
 *
 * @param[in] regAddr   : register address
 * @param[in] u8SrcData : data which user attempt to transfer through SPI bus
 * @param[in] u32TransCount : transfer count
 * @return none
 */
void setBLE_SpiPDMATx(uint8_t regAddr, uint32_t u32SrcAddr, uint32_t u32TransCount)
{
    uint32_t u32i;

    while (1)
    {
        if (SPI_IS_BUSY(SPI0) == 0)
        {
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
            __NOP();
#else
            setBLE_SpiSSHigh();
            __NOP();
            __NOP();
            setBLE_SpiSSLow();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
            if (((PDMA_GET_INT_STATUS(PDMA)) & (PDMA_INTSTS_TDIF_Msk | PDMA_INTSTS_ABTIF_Msk | PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk)))
            {
                u32i = (PDMA_GET_TD_STS(PDMA) & SPI_OPENED_CH);
                if (u32i)   //Check the PDMA transfer done flags
                {
                    PDMA_CLR_TD_FLAG(PDMA, u32i);   //Clear the PDMA transfer done flags
                }
            }
            break;
        }
    }
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].SA = u32SrcAddr;
    SPI_WRITE_TX(SPI0, (regAddr & 0x7F));   //write 1st byte: regAddr & 0x7F

    if ((regAddr & 0x80))
    {
        SPI_WRITE_TX(SPI0, 0x01);   //write 2nd byte: (regAddr & 0x80) >> 7
    }
    else
    {
        SPI_WRITE_TX(SPI0, 0x00);
    }

    if (regAddr == TX_BUFFER_WRITE_PORT)    //if (writing to TX_buffer)
    {
        uint32_t ram_start_addr;

        ram_start_addr = getBLE_TxFIFOAddr();
        SPI_WRITE_TX(SPI0, (ram_start_addr & 0xFF));                    //write 3rd byte: ram_start_addr & 0xFF
        SPI_WRITE_TX(SPI0, ((ram_start_addr & 0x0100) >> 8) | 0x80);    //write 4th byte: (ram_start_addr & 0x0100)>>8 | (b7<<7),  b7=1/0 means fill payload/header
    }

    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL &= ~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_TXWIDTH_Msk | PDMA_DSCT_CTL_OPMODE_Msk);
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= ((PDMA_WIDTH_8 | PDMA_OP_BASIC) | ((u32TransCount - 1UL) << PDMA_DSCT_CTL_TXCNT_Pos));

    SPI_TRIGGER_TX_PDMA(SPI0);  //Enable SPI master DMA function
}



/** This function is used to manual set SPI SS pin to high state immediately.
 *
 * @return none
 */
void setBLE_SpiSSHigh(void)
{
    SPI_SET_SS_HIGH(SPI0);
}


/** This function is used to manual set SPI SS pin to low state immediately.
 *
 * @return none
 */
void setBLE_SpiSSLow(void)
{
    SPI_SET_SS_LOW(SPI0);
}


/** This function is used to manual set SPI SS pin to low state immediately.
 *
 * @return none
 */
void setBLE_SpiWaitAndSSHigh(void)
{
#if (SPI_SS_CONTROL == AUTO_SPI_SS)
    __NOP();
#else
    while (SPI_IS_BUSY(SPI0) == 1);   //busy
    setBLE_SpiSSHigh();
#endif  //if (SPI_SS_CONTROL == AUTO_SPI_SS)
}

#if defined (__CC_ARM)
#pragma pop
#endif

