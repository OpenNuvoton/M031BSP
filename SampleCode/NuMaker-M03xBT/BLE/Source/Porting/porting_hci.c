#include "porting_hci.h"
#include "string.h"
#include "ble_cmd.h"
#include "ble_hci_cmd.h"

#define HCI_PKT_COMMAND                         0x01
#define HCI_PKT_ACL_DATA                        0x02
#define HCI_PKT_SYNCHRONOUS_DATA                0x03
#define HCI_PKT_EVENT                           0x04

#define FLD_HCI_PKT_LTH_COMMAND                 3       //0:Packet Type, 1,2:OpCode
#define FLD_HCI_PKT_LTH_ACL_DATA                3       //0:Packet Type, 1,2:Handle & Flags
#define FLD_HCI_PKT_LTH_SYNCHRONOUS_DATA        3       //0:Packet Type, 1,2:Connection_Handle & Flag
#define FLD_HCI_PKT_LTH_EVENT                   2       //0:Packet Type, 1:Event Code

/******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t UARTBufferT[260];   //UART buffer for Tx
uint8_t UARTBufferR[260];   //UART buffer for Rx
uint16_t UARTBufIdxR;    //UART buffer index for Rx
uint16_t UARTBufIdxT;    //UART buffer index for Tx
uint16_t UART_Tx_size;
uint16_t UART_Rx_size;
uint8_t UARTBufValidT;


/******************************************************************************
 * Public Functions
 ******************************************************************************/
/* Init UART */
void UART_Init(void)
{
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Enable UART RDA and THRE interrupt */
    //UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk));

    /* Enable UART RDA interrupt, THRE interrupt should enable until the UART Tx start */
    UART_EnableInt(UART0, UART_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(UART02_IRQn);

    /* Set RX Trigger Level = 1 */
    UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;

    UARTBufIdxR = 0;                    //initial
    UARTBufIdxT = 0;                    //initial
    UARTBufValidT = 0;                  //initial
}


/* UART_TX */
void setUART_Tx(uint8_t *bufAddr, uint16_t length)
{
    UART_Tx_size = length;
    memcpy(UARTBufferT, bufAddr, length);
    //SBUF = UARTBufferT[0];
    UART_WRITE(UART0, UARTBufferT[0]);
    UARTBufValidT = 1;
    UARTBufIdxT = 1;
    UART_ENABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
}

/* UART Interrupt Handler: HCI command decoder */
void UART02_IRQHandler(void)
{
    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk))            //Received data
    {
        //RDAINT Flag Cleared By Read UART_DAT
        UARTBufferR[UARTBufIdxR] = UART_READ(UART0);
        if (UARTBufIdxR < 4)
        {
            switch (UARTBufIdxR)                                                        //HCI__005
            {
            case 3:
                if (UARTBufferR[0] == HCI_PKT_COMMAND)
                {
                    UART_Rx_size = UARTBufferR[FLD_HCI_PKT_LTH_COMMAND] + 3;           //2byte OpCode + 1Byte Parameter Length
                }
                else if (UARTBufferR[0] == HCI_PKT_ACL_DATA)
                {
                    UART_Rx_size = UARTBufferR[FLD_HCI_PKT_LTH_ACL_DATA] + 4;          //2byte Handle + 2Byte Data Length
                }
                else                                                                   //if(mblk->Para.Data[0] == HCI_PKT_SYNCHRONOUS_DATA)
                {
                    UART_Rx_size = UARTBufferR[FLD_HCI_PKT_LTH_SYNCHRONOUS_DATA] + 3;  //2byte Handle + 1Byte Parameter Length
                }
                break;

            case 2:
                if (UARTBufferR[0] == HCI_PKT_EVENT)
                {
                    UART_Rx_size = UARTBufferR[FLD_HCI_PKT_LTH_EVENT] + 2;              //1byte EventCode + 1Byte Parameter Length
                }
                break;

            default:
                UART_Rx_size = 4;
                break;
            }
        }
        UARTBufIdxR++;

        if (UARTBufIdxR > UART_Rx_size)
        {
            setBLE_HCICommand(&UARTBufferR[0], UARTBufIdxR);
            UARTBufIdxR = 0;    //release for next packet receive
        }
    }
    if (UART_GET_INT_FLAG(UART0, UART_INTSTS_THREINT_Msk))
    {
        //THERINT Flag Cleared By Write UART_DAT
        if (UARTBufValidT == 1)                 //UART buffer in use
        {
            if (UARTBufIdxT == UART_Tx_size)
            {
                /* No more data, just stop Tx (Stop work) */
                UART_DISABLE_INT(UART0, UART_INTEN_THREIEN_Msk);
                UARTBufIdxT = 0;        //initial
                UARTBufValidT = 0;      //initial
            }
            else
            {
                UART_WRITE(UART0, UARTBufferT[UARTBufIdxT]);
                UARTBufIdxT++;
            }
        }
    }
}  //end of UART02_IRQHandler()

