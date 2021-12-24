#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "ble_dtm.h"
#include "ble_rftone.h"
#include "rf_phy.h"
#include "porting_misc.h"
#include "porting_spi.h"
#include "porting_dtm.h"
#include "porting_rfpower.h"


/**************************************************************************
* Definitions
**************************************************************************/
#define BLEDTM_RADIO_PHY_1M         (0UL)
#define BLEDTM_RADIO_PHY_2M         (1UL)

#define BLEDTM_PHYS_CH_MIN          0       /**< Minimum number of valid channels in BLE. */
#define BLEDTM_PHYS_CH_MAX          39      /**< Maximum number of valid channels in BLE. */

#define BLEDTM_HEADER_OFFSET        0                                               /**< Index where the header of the pdu is located. */
#define BLEDTM_HEADER_SIZE          2                                               /**< Size of PDU header. */
#define BLEDTM_PAYLOAD_MAX_SIZE     255                                             /**< Maximum payload size allowed during dtm execution. */
#define BLEDTM_LENGTH_OFFSET        (BLEDTM_HEADER_OFFSET + 1)                      /**< Index where the length of the payload is encoded. */
#define BLEDTM_PDU_MAX_MEMORY_SIZE  (BLEDTM_HEADER_SIZE + BLEDTM_PAYLOAD_MAX_SIZE)  /**< Maximum PDU size allowed during dtm execution. */
#define BLEDTM_PREAMBLE_SIZE        1
#define BLEDTM_ACCESS_ADDR_SIZE     4
#define BLEDTM_CRC_SIZE             3
#define BLEDTM_PHY_SIZE_MAX         (BLEDTM_PREAMBLE_SIZE + BLEDTM_ACCESS_ADDR_SIZE + BLEDTM_HEADER_SIZE + BLEDTM_PAYLOAD_MAX_SIZE + BLEDTM_CRC_SIZE)



// Values that for now are "constants" - they could be configured by a function setting them,
// but most of these are set by the BLE DTM standard, so changing them is not relevant.
#define RFPHY_TEST_0X0F_REF_PATTERN  0x0f  /**<  RF-PHY test packet patterns, for the repeated octet packets. */
#define RFPHY_TEST_0X55_REF_PATTERN  0x55  /**<  RF-PHY test packet patterns, for the repeated octet packets. */
#define RFPHY_TEST_0XFF_REF_PATTERN  0xFF  /**<  RF-PHY test packet patterns, for the repeated octet packets. */


/** The PRBS9 sequence used as packet payload. \n
The bytes in the sequence is in the right order, but the bits of each byte in the array is reverse.
of that found by running the PRBS9 algorithm. This is because of the endianess of the nRF5 radio. */
#define PRBS9_CONTENT  {0xFF, 0xC1, 0xFB, 0xE8, 0x4C, 0x90, 0x72, 0x8B,   \
                        0xE7, 0xB3, 0x51, 0x89, 0x63, 0xAB, 0x23, 0x23,   \
                        0x02, 0x84, 0x18, 0x72, 0xAA, 0x61, 0x2F, 0x3B,   \
                        0x51, 0xA8, 0xE5, 0x37, 0x49, 0xFB, 0xC9, 0xCA,   \
                        0x0C, 0x18, 0x53, 0x2C, 0xFD, 0x45, 0xE3, 0x9A,   \
                        0xE6, 0xF1, 0x5D, 0xB0, 0xB6, 0x1B, 0xB4, 0xBE,   \
                        0x2A, 0x50, 0xEA, 0xE9, 0x0E, 0x9C, 0x4B, 0x5E,   \
                        0x57, 0x24, 0xCC, 0xA1, 0xB7, 0x59, 0xB8, 0x87,   \
                        0xFF, 0xE0, 0x7D, 0x74, 0x26, 0x48, 0xB9, 0xC5,   \
                        0xF3, 0xD9, 0xA8, 0xC4, 0xB1, 0xD5, 0x91, 0x11,   \
                        0x01, 0x42, 0x0C, 0x39, 0xD5, 0xB0, 0x97, 0x9D,   \
                        0x28, 0xD4, 0xF2, 0x9B, 0xA4, 0xFD, 0x64, 0x65,   \
                        0x06, 0x8C, 0x29, 0x96, 0xFE, 0xA2, 0x71, 0x4D,   \
                        0xF3, 0xF8, 0x2E, 0x58, 0xDB, 0x0D, 0x5A, 0x5F,   \
                        0x15, 0x28, 0xF5, 0x74, 0x07, 0xCE, 0x25, 0xAF,   \
                        0x2B, 0x12, 0xE6, 0xD0, 0xDB, 0x2C, 0xDC, 0xC3,   \
                        0x7F, 0xF0, 0x3E, 0x3A, 0x13, 0xA4, 0xDC, 0xE2,   \
                        0xF9, 0x6C, 0x54, 0xE2, 0xD8, 0xEA, 0xC8, 0x88,   \
                        0x00, 0x21, 0x86, 0x9C, 0x6A, 0xD8, 0xCB, 0x4E,   \
                        0x14, 0x6A, 0xF9, 0x4D, 0xD2, 0x7E, 0xB2, 0x32,   \
                        0x03, 0xC6, 0x14, 0x4B, 0x7F, 0xD1, 0xB8, 0xA6,   \
                        0x79, 0x7C, 0x17, 0xAC, 0xED, 0x06, 0xAD, 0xAF,   \
                        0x0A, 0x94, 0x7A, 0xBA, 0x03, 0xE7, 0x92, 0xD7,   \
                        0x15, 0x09, 0x73, 0xE8, 0x6D, 0x16, 0xEE, 0xE1,   \
                        0x3F, 0x78, 0x1F, 0x9D, 0x09, 0x52, 0x6E, 0xF1,   \
                        0x7C, 0x36, 0x2A, 0x71, 0x6C, 0x75, 0x64, 0x44,   \
                        0x80, 0x10, 0x43, 0x4E, 0x35, 0xEC, 0x65, 0x27,   \
                        0x0A, 0xB5, 0xFC, 0x26, 0x69, 0x3F, 0x59, 0x99,   \
                        0x01, 0x63, 0x8A, 0xA5, 0xBF, 0x68, 0x5C, 0xD3,   \
                        0x3C, 0xBE, 0x0B, 0xD6, 0x76, 0x83, 0xD6, 0x57,   \
                        0x05, 0x4A, 0x3D, 0xDD, 0x81, 0x73, 0xC9, 0xEB,   \
                        0x8A, 0x84, 0x39, 0xF4, 0x36, 0x0B, 0xF7}


/** Pseudo-random bit sequence defined by the BLE standard. */
static uint8_t const     bledtm_prbs_content[]    = PRBS9_CONTENT;


/**@brief States used for the DTM test implementation.
 */
typedef enum
{
    BLEDTM_STATE_UNINITIALIZED    = 0,    /**< The DTM is uninitialized. */
    BLEDTM_STATE_IDLE             = 1,    /**< State when system has just initialized, or current test has completed. */
    BLEDTM_STATE_TRANSMITTER_TEST = 2,    /**< State used when a DTM Transmission test is running. */
    BLEDTM_STATE_CARRIER_TEST     = 3,    /**< State used when a DTM Carrier test is running (Vendor specific test). */
    BLEDTM_STATE_RECEIVER_TEST    = 4     /**< State used when a DTM Receive test is running. */
} BLEDTM_STATE;


/**@brief Structure holding the PDU used for transmitting/receiving a PDU.
 */
typedef struct
{
    uint8_t content[BLEDTM_HEADER_SIZE + BLEDTM_PAYLOAD_MAX_SIZE];  /**< PDU packet content. */
} BLEDTM_pdu_type;


typedef struct
{
    uint8_t content[4 + BLEDTM_PHY_SIZE_MAX];   /**BLEDTM_PHY_SIZE_MAX+buf_addr1+buf_addr2+< Air packet content.>+dummy */
} BLEDTM_airPacket_type;


/**************************************************************************
* Variables
**************************************************************************/
static BLEDTM_STATE      bledtm_state                   = BLEDTM_STATE_UNINITIALIZED;   /**< Current machine state. */
static uint8_t           bledtm_radio_mode              = BLEDTM_RADIO_PHY_1M;          /**< Radio mode. */
static uint32_t          bledtm_txInterval_us           = 2500;                         /**< Time between start of Tx packets(in us), initial value depends on packet length. */
static uint32_t          bledtm_current_timerCounter    = 0;                            /**< Timer counter to check DTM byte to byte time difference. */

static uint8_t           bledtm_txPower                 = TXPOWER_9DBM;                 /**< TX power setting. */

static uint8_t           bledtm_timerReport_flag        = 0;                            /**< Flag of RX timer has reported. */
static uint8_t           bledtm_rxComplete_flag         = 0;                            /**< Flag of RX completed. */
static uint8_t           bledtm_eventGet_flag;                                          /**< Flag of BLE DTM event, 0: command has been processed, 1: not yet reported event bytes. */


static uint16_t          bledtm_rx_pkt_count            = 0;                            /**< Number of valid packets received. */
static uint8_t           bledtm_crc_status              = 0;                            /**< CRC State: 1:RX crc_check ok. */


static BLEDTM_pdu_type   bledtm_pdu;                                                    /**< PDU to be sent. */
static uint16_t          bledtm_event;                                                  /**< current command status - initially "ok", may be set if error detected, or to packet count. */

static uint32_t          bledtm_packet_length;                                          /**< Payload length of transmitted PDU, bits 2:7 of 16-bit dtm command. */
static uint32_t          bledtm_packet_type;                                            /**< Bits 0..1 of 16-bit transmit command, or 0xFFFFFFFF. */
static uint32_t          bledtm_phys_ch;                                                /**< 0..39 physical channel number (base 2402 MHz, Interval 2 MHz), bits 8:13 of 16-bit dtm command. */


static BLEDTM_airPacket_type  bledtm_air_txPacket;                                      /**< Air TX Packet. */
static BLEDTM_airPacket_type  bledtm_air_rx_packet;                                     /**< Air RX Packet. */

static uint8_t           bledtm_proprietary_Tx_ToneMode_flag     = 0;                   /**< Flag of Tx output single tone mode. */

#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
// command for proprietary tool (Lab Test Tool).
//==========================================================================
static uint8_t           bledtm_proprietary_guiGetRssi_flag = 0;

static uint8_t           bledtm_proprietary_physTx_ChRandom_flag = 0;
static uint8_t           bledtm_proprietary_physTx_PackageNumberByUser_flag = 0;
static uint16_t          bledtm_proprietary_Tx_UserDef_PackageNumber_target = 0;
static uint16_t          bledtm_proprietary_Tx_UserDefPackageNumber_real = 0;
#endif


/**************************************************************************
 * Function Prototype Declaration
 **************************************************************************/
static uint32_t bledtm_radio_init(void);
static uint8_t  bledtm_pdu_check(void);
static uint32_t bledtm_cmd(uint32_t cmd, uint32_t freq, uint32_t length, uint32_t payload);

/**************************************************************************
* Public Function
**************************************************************************/

/**Function for initializing DTM module.
 *
 * @retval BLEDTM_ERROR_ILLEGAL_CONFIGURATION : Invalid configurations.
 * @retval BLEDTM_SUCCESS                     : Setting success.
*/
BleDtmStatus BleDTM_init(void)
{
    // init BLE DTM timer
    setDTM_timer_init();

    // init radio
    if ((bledtm_radio_init() != BLEDTM_SUCCESS))
    {
        return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
    }

    // init variables
    bledtm_current_timerCounter = 0;
    bledtm_eventGet_flag        = false;
    bledtm_state                = BLEDTM_STATE_IDLE;
    bledtm_packet_length        = 0;

    /*
    Send Event on Pending bit:
    0 = only enabled interrupts or events can wakeup the processor, disabled interrupts are excluded
    1 = enabled events and all interrupts, including disabled interrupts, can wakeup the processor.
    When an event or interrupt enters pending state, the event signal wakes up the processor from WFE. If the processor is not waiting for an event, the event is registered and affects the next WFE.
    The processor also wakes up on execution of an SEV instruction or an external event.
    */
    return BLEDTM_SUCCESS;
}


/**Function for handle radio control process.
 *
 * @note handle radio first, to give it highest priority:
 *
 */
void BleDTM_radio_process(void)
{
    // Event may be the reception of a packet -
    if (bledtm_rxComplete_flag == 1)
    {
        bledtm_rxComplete_flag = 0;

        if (bledtm_state == BLEDTM_STATE_RECEIVER_TEST)
        {
            if ((bledtm_crc_status == 1) && bledtm_pdu_check())  // CRC is ok, so the entire payload is received
            {
                bledtm_crc_status = 0;
                // Count the number of successfully received packets
                bledtm_rx_pkt_count++;
            }
            // Note that failing packets are simply ignored (CRC or contents error).

            // Zero fill all pdu fields to avoid stray data
            memset(&bledtm_pdu, 0, BLEDTM_PDU_MAX_MEMORY_SIZE);   // max size: 2+255=257
        }
    }
}


/** Function for splitting UART command bit fields into separate command parameters for the DTM library.
 *
 * @param[in]   command : The packed UART command.
 *
 * @return      result status.
 */
uint32_t BleDTM_cmd_process(uint16_t command)
{
    uint32_t result;

    uint32_t       command_code = (command >> 14) & 0x03;
    uint32_t       freq         = (command >> 8) & 0x3F;
    uint32_t       length       = (command >> 2) & 0x3F;
    uint32_t       payload      = command & 0x03;

    result = bledtm_cmd(command_code, freq, length, payload);

    return result;
}


/** Function for reading the result of a DTM command.
 *
 * @param[out] p_dtm_event : Pointer to buffer for 16 bit event code according to DTM standard.
 *
 * @return  Checking result.
 * @retval  0:  no event since last call.
 * @retval  1:  new event.
 */
uint8_t BleDTM_event_get(uint32_t *dtm_event)
{
    uint8_t was_new = bledtm_eventGet_flag;  //set to true in bledtm_cmd()

    // mark the current event as retrieved
    bledtm_eventGet_flag  = false;
    *dtm_event = bledtm_event;

    // return value indicates whether this value was already retrieved.
    return was_new;
}


/** Function for get current timer counter.
 *
 * @return  timer counter.
 */
uint32_t BleDTM_timerCount_update(void)
{
    if (bledtm_timerReport_flag == 1)
    {
        bledtm_timerReport_flag = 0;
        return ++bledtm_current_timerCounter;    // only return at here (every 260us) /
    }

    return bledtm_current_timerCounter;
}

/**************************************************************************
* Private Function
**************************************************************************/
/**@brief Function for turning off the radio after a test.
 *        Also called after test done, to be ready for next test.
 */
static void bledtm_radio_reset(void)
{
    // reset rf phy
    BleDTM_phy_reset();

    // clear received packet count
    bledtm_rx_pkt_count = 0;
}


/**@brief Function for initializing the radio for DTM.
 */
static uint32_t bledtm_radio_init(void)
{
    // reset rf phy before configuring it
    bledtm_radio_reset();

    // int rf phy
    setRF_Init(DCDC_REGULATOR, XTAL_16M);

    // set default TX power
    BleDTM_txPower_set(bledtm_txPower);

    return BLEDTM_SUCCESS;
}


/**@brief Function for terminating the ongoing test (if any) and closing down the radio.
 */
static void bledtm_test_done(void)
{
    //TX/RX_close & bledtm_rx_pkt_count=0

    // reset rf phy
    bledtm_radio_reset();

    // set to idle state
    bledtm_state = BLEDTM_STATE_IDLE;
}


static uint8_t bledtm_pdu_check(void)
{
    uint8_t        k;                // Byte pointer for running through PDU payload
    uint8_t        pattern;          // Repeating octet value in payload
    uint32_t       pdu_packet_type;  // Note: PDU packet type is a 4-bit field in HCI, but 2 bits in BLE DTM
    uint32_t       length = 0;

    pdu_packet_type = (uint32_t)(bledtm_pdu.content[BLEDTM_HEADER_OFFSET] & 0x0F); // from received data
    length = bledtm_packet_length;

    // Check that the length is valid.
    if (length > BLEDTM_PAYLOAD_MAX_SIZE)
    {
        return false;
    }

    //If the 1Mbit or 2Mbit radio mode is active, check that one of the three valid uncoded DTM packet types are selected.
    if ((bledtm_radio_mode == BLEDTM_RADIO_PHY_1M || bledtm_radio_mode == BLEDTM_RADIO_PHY_2M) && (pdu_packet_type > (uint32_t)DTM_PKT_0X55))
    {
        return false;
    }

    if (pdu_packet_type == DTM_PKT_PRBS9)
    {
        //Payload does not consist of one repeated octet; must compare ir with entire block into
        return (memcmp(bledtm_pdu.content + BLEDTM_HEADER_SIZE, bledtm_prbs_content, length) == 0);
    }

    if (pdu_packet_type == DTM_PKT_0X0F)
    {
        pattern = RFPHY_TEST_0X0F_REF_PATTERN;
    }
    else if (pdu_packet_type == DTM_PKT_0X55)
    {
        pattern = RFPHY_TEST_0X55_REF_PATTERN;
    }
    else if (pdu_packet_type == DTM_PKT_0XFF)
    {
        pattern = RFPHY_TEST_0XFF_REF_PATTERN;
    }
    else
    {
        //No valid packet type set.
        return false;
    }

    for (k = 0; k < length; k++)
    {
        //Check repeated pattern filling the PDU payload
        if (bledtm_pdu.content[k + 2] != pattern)
        {
            return false;
        }
    }
    return true;
}



static uint32_t bledtm_packetInterval_calculate(uint32_t test_payload_length, uint32_t mode)
{
    uint32_t test_packet_length = 0; // [us] NOTE: bits are us at 1Mbit
    uint32_t packet_interval    = 0; // us
    uint32_t overhead_bits      = 0; // bits

    uint32_t i       = 0;
    uint32_t timeout = 0;

    /* packet overhead
     * see BLE [Vol 6, Part F] page 213
     * 4.1 LE TEST PACKET FORMAT */
    if (mode == BLEDTM_RADIO_PHY_2M)
    {
        // 16 preamble
        // 32 sync word
        //  8 PDU header, actually packetHeaderS0len * 8
        //  8 PDU length, actually packetHeaderLFlen
        // 24 CRC
        overhead_bits = 88; // 11 bytes
    }
    else if (mode == BLEDTM_RADIO_PHY_1M)
    {
        //  8 preamble
        // 32 sync word
        //  8 PDU header, actually packetHeaderS0len * 8
        //  8 PDU length, actually packetHeaderLFlen
        // 24 CRC
        overhead_bits = 80; // 10 bytes
    }

    /* add PDU payload test_payload length */
    test_packet_length = (test_payload_length * 8); // in bits

    // add overhead calculated above
    test_packet_length += overhead_bits;
    // we remember this bits are us in 1Mbit

    if (mode == BLEDTM_RADIO_PHY_2M)
    {
        test_packet_length /= 2; // double speed
    }

    /*
     * packet_interval = ceil((test_packet_length+249)/625)*625
     * NOTE: To avoid floating point an equivalent calculation is used.
     */
    do
    {
        i++;
        timeout = i * 625;
    } while (test_packet_length + 249 > timeout);
    packet_interval = i * 625;

    return packet_interval;
}


#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
/**@brief Function for Get the RSSI flag.
 *
 * @retval 0: No RSSI value provided.
 * @retval 1: The last RSSI value will be provided after BleDTM_event_get.
*/
uint8_t BleDTM_gui_rssiFlag_get(void)
{
    return bledtm_proprietary_guiGetRssi_flag;
}



// command for proprietary tool (Lab Test Tool).
void bledtm_cmd_proprietary_setup_reset()
{
    //reset flag
    bledtm_proprietary_physTx_ChRandom_flag = 0;
    bledtm_proprietary_physTx_PackageNumberByUser_flag = 0; //0:unlimited, 1:package number by user
    bledtm_proprietary_Tx_UserDef_PackageNumber_target = 0;
    bledtm_proprietary_Tx_UserDefPackageNumber_real = 0;
}


// command for proprietary tool (Lab Test Tool).
static void  bledtm_cmd_proprietary_tool(uint32_t freq, uint32_t length, uint32_t payload)
{
    switch (freq)
    {
    case DTM_VENDORSPECIFIC_TX_RANDOM_CH_DISABLE: //Random CH Disable
        bledtm_proprietary_physTx_ChRandom_flag = 0;
        break;

    case DTM_VENDORSPECIFIC_TX_RANDOM_CH_ENABLE:  //Random CH enable
        bledtm_proprietary_physTx_ChRandom_flag = 1;
        break;

    case DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_UNLIMITED:  // package number unlimited
        bledtm_proprietary_physTx_PackageNumberByUser_flag = 0; //unlimited
        bledtm_proprietary_Tx_UserDef_PackageNumber_target = 0;
        break;

    case DTM_VENDORSPECIFIC_TX_PACKAGE_NUMBER_BY_USER:  //package number by user
        bledtm_proprietary_physTx_PackageNumberByUser_flag = 1; //by user
        bledtm_proprietary_Tx_UserDef_PackageNumber_target = ((((length & 0x3F) << 2) | (payload & 0x03)) * 10);
        break;

    case DTM_VENDORSPECIFIC_TX_POWER_SELECT:
        BleDTM_txPower_set((uint8_t)length);//40pin only
        bledtm_txPower = (uint8_t)length;
        break;

    case DTM_VENDORSPECIFIC_RX_GET_RSSI:
        bledtm_proprietary_guiGetRssi_flag = (uint8_t)payload; //payload => 0:no print RSSI, 1:print RSSI after report PER.
        break;

    case DTM_VENDORSPECIFIC_TX_POWER_SELECT_MORE_OPTION:
        BleDTM_txPower_set((uint8_t)length);//40pin only
        bledtm_txPower = (uint8_t)length;
        break;

    case DTM_VENDORSPECIFIC_GO_SLEEP_MODE:
        BleDTM_sleepMode_set(BLEDTM_SLEEP_MODE);
        break;

    case DTM_VENDORSPECIFIC_GO_DEEP_SLEEP_MODE:
        BleDTM_sleepMode_set(BLEDTM_DEEP_SLEEP_MODE);
        break;
    case DTM_VENDORSPECIFIC_GO_TXRF_CONTINUOUS_MODE:
        bledtm_proprietary_Tx_ToneMode_flag = 1;
        break;
    case DTM_VENDORSPECIFIC_GO_TXRF_DTM_MODE:
        bledtm_proprietary_Tx_ToneMode_flag = 0;
        break;
    default:
        bledtm_proprietary_physTx_ChRandom_flag = 0;
        bledtm_proprietary_physTx_PackageNumberByUser_flag = 0; //unlimited
        bledtm_proprietary_Tx_UserDef_PackageNumber_target = 0;
        bledtm_proprietary_Tx_ToneMode_flag = 0;
        break;
    }
}
#endif // (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)   


static uint32_t bledtm_cmd(uint32_t cmd, uint32_t freq, uint32_t length, uint32_t payload)
{
    // Save specified packet in uint8_t variable for tx/rx functions to use.
    // Note that BLE conformance testers always use full length packets.
    bledtm_packet_length = (bledtm_packet_length & 0xC0) | ((uint8_t)length & 0x3F); //* payload length information from CMD (not include header length) */
    bledtm_packet_type   = payload;  //* type information from CMD */
    bledtm_phys_ch       = freq;     //* frequency information from CMD */

    // Clean out any non-retrieved event that might linger from an earlier test
    bledtm_eventGet_flag  = true;

    // Set default event; any error will set it to LE_TEST_STATUS_EVENT_ERROR
    bledtm_event    = LE_TEST_STATUS_EVENT_SUCCESS;

    if (bledtm_state == BLEDTM_STATE_UNINITIALIZED)
    {
        // Application has not explicitly initialized DTM,
        return BLEDTM_ERROR_UNINITIALIZED;
    }
    //Byte1 = cmd(7:6) + freq(5:0)
    //Byte2 = lengte(7:2) + payload(1:0)
    if (cmd == LE_TEST_SETUP)  //0x00
    {
        //* freq = control 6 bits     */
        //* length = parameter 6 bits */

        // Note that timer will continue running after a reset
        bledtm_test_done();

        switch (freq)
        {
        case LE_TEST_SETUP_RESET: /* control = 0x00 (Reset) */
            if (length != 0x00)
            {
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
            }

            // Reset the packet length upper bits.
            bledtm_packet_length = 0;

            // Reset the selected PHY to 1Mbit
            bledtm_radio_mode   = BLEDTM_RADIO_PHY_1M;

#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
            // command for proprietary tool (Lab Test Tool).
            bledtm_cmd_proprietary_setup_reset();
#endif
            break;

        case LE_TEST_SETUP_SET_UPPER: /* control = 0x01 */
            // parameter = 0x00~0x03 (set upper 2 bits of data parameter)
            if (length > 0x03)
            {
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            bledtm_packet_length = length << 6;
            break;

        case LE_TEST_SETUP_SET_PHY:  /* control = 0x02 */
            switch (length)      //* parameter */
            {
            case LE_PHY_1M:  //* 0x01 (set PHY to LE 1M) */
                bledtm_radio_mode   = BLEDTM_RADIO_PHY_1M;
                return bledtm_radio_init();

            case LE_PHY_2M:  //* 0x02 (set PHY to LE 2M) */
                bledtm_radio_mode   = BLEDTM_RADIO_PHY_2M;
                return bledtm_radio_init();

            case LE_PHY_LE_CODED_S8:  //* 0x03 (set PHY to LE coded S=8) */
            case LE_PHY_LE_CODED_S2:  //* 0x04 (set PHY to LE Coded S=2) */
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;

            default:
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
            }

        case LE_TEST_SETUP_SELECT_MODULATION: /* control = 0x03 */
            /* parameter = 0x00~0x01 (assume TX has modulation index) */
            if (length > 0x01)
            {
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            // Only standard modulation is supported.
            break;

        case LE_TEST_SETUP_READ_SUPPORTED:  /* control = 0x04 */
            /* parameter = 0x00 (read test case support features) */
            if (length != 0x00)
            {
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            bledtm_event = 0x0006;
            break;

        case LE_TEST_SETUP_READ_MAX:  /* control = 0x05 */
            // Read max supported value.
            switch (length)
            {
            case 0x00:  // Read supportedMaxTxOctets
            case 0x02:  // Read supportedMaxRxOctets
                bledtm_event = 0x01FE;
                break;
            case 0x01:  // Read supportedMaxTxTime
            case 0x03:  // Read supportedMaxRxTime
                bledtm_event = 0x4290;
                break;
            default:
                bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
                return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
            }
            break;

        default:
            bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
            return BLEDTM_ERROR_ILLEGAL_CONFIGURATION;
        }
        return BLEDTM_SUCCESS;
    }

    if (cmd == LE_TEST_END) //0x03
    {

#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
        if (freq != 0)
        {
            // command for proprietary tool (Lab Test Tool).
            bledtm_cmd_proprietary_tool(freq, length, payload);
            return BLEDTM_SUCCESS;
        }
#endif

        if (bledtm_state == BLEDTM_STATE_IDLE)
        {
            // Sequencing error - only rx or tx test may be ended!
            bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
            return BLEDTM_ERROR_INVALID_STATE;
        }

#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
        bledtm_proprietary_physTx_ChRandom_flag = 0;
#endif

        bledtm_event = LE_PACKET_REPORTING_EVENT | bledtm_rx_pkt_count;   // m_rx_pkt_count=0 in bledtm_radio_reset()
        bledtm_test_done();


        return BLEDTM_SUCCESS;
    }

    if (bledtm_state != BLEDTM_STATE_IDLE)
    {
        // Sequencing error - only TEST_END/RESET are legal while test is running
        // Note: State is unchanged; ongoing test not affected
        bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
        return BLEDTM_ERROR_INVALID_STATE;
    }

    /*******************************************************************************
    *   DTM Parameter Check
    *******************************************************************************/
    // Check for illegal values of bledtm_phys_ch. Skip the check if the packet is vendor specific.
    if (bledtm_phys_ch > BLEDTM_PHYS_CH_MAX)  //>39
    {
        // Parameter error
        // Note: State is unchanged; ongoing test not affected
        bledtm_event = LE_TEST_STATUS_EVENT_ERROR;

        return BLEDTM_ERROR_ILLEGAL_CHANNEL;
    }

    /*******************************************************************************
    *   RX
    *******************************************************************************/
    if (cmd == LE_RECEIVER_TEST)    //0x01
    {
        // reset bledtm_rx_pkt_count
        bledtm_rx_pkt_count = 0;

        // zero fill all pdu fields to avoid stray data from earlier test run
        memset(&bledtm_pdu, 0, BLEDTM_PDU_MAX_MEMORY_SIZE);

        // reinitialize "everything"; RF interrupts OFF
        BleDTM_radio_prepare(BLEDTM_RX_MODE, bledtm_packet_type, bledtm_radio_mode, bledtm_phys_ch);

        // DTM test payload length
        BleDTM_testPayload_set(bledtm_packet_length);

        // set state
        bledtm_state = BLEDTM_STATE_RECEIVER_TEST;

        return BLEDTM_SUCCESS;
    }

    /*******************************************************************************
    *   TX
    *******************************************************************************/
    if (cmd == LE_TRANSMITTER_TEST)  //0x02
    {
        // Check for illegal values of m_packet_length. Skip the check if the packet is vendor specific.
        if (bledtm_packet_length > BLEDTM_PAYLOAD_MAX_SIZE)   //>255
        {
            // Parameter error
            bledtm_event = LE_TEST_STATUS_EVENT_ERROR;
            return BLEDTM_ERROR_ILLEGAL_LENGTH;
        }

        // Note that PDU uses 4 bits even though BLE DTM uses only 2 (the HCI SDU uses all 4)
        bledtm_pdu.content[BLEDTM_HEADER_OFFSET] = ((uint8_t)bledtm_packet_type & 0x0F);
        bledtm_pdu.content[BLEDTM_LENGTH_OFFSET] = bledtm_packet_length;

        //pack whole air packet
        switch (bledtm_packet_type)
        {
        case DTM_PKT_PRBS9:
        case DTM_PKT_0X0F:
        case DTM_PKT_0X55:
        case DTM_PKT_0XFF:
            memcpy(bledtm_air_txPacket.content, bledtm_pdu.content, BLEDTM_HEADER_SIZE); //DTM mode: only write header (2 bytes)
            BleDTM_testPayload_set(bledtm_packet_length);                         //DTM payload length
            break;
        default:
            break;
        }

        if (bledtm_proprietary_Tx_ToneMode_flag == 0) //DTM mode
        {
            // Initialize CRC value, set channel.
            BleDTM_radio_prepare(BLEDTM_TX_MODE, bledtm_packet_type, bledtm_radio_mode, bledtm_phys_ch);

            // Set the timer to the correct period. The delay between each packet is described in the
            // Bluetooth Core Specification version 4.2 Vol. 6 Part F Section 4.1.6.
            TIMER_Stop(TIMER0);       //stop timer counting
            TIMER_DisableInt(TIMER0); //disable timer interrupt

            bledtm_txInterval_us = bledtm_packetInterval_calculate(bledtm_packet_length, bledtm_radio_mode);  // new, I(L)=N*625us


            TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000 / bledtm_txInterval_us); // per interrupt
            TIMER_EnableInt(TIMER0);  //enable timer interrupt
            TIMER_Start(TIMER0);      //start timer counting
        }
        else
        {
            setRF_MPTestContTx(bledtm_phys_ch);
        }

        // set state
        bledtm_state  = BLEDTM_STATE_TRANSMITTER_TEST;

    } //end of if(cmd==LE_TRANSMITTER_TEST)
    return BLEDTM_SUCCESS;
}



/**************************************************************************
* Interrupt Service Routine
**************************************************************************/
void TMR0_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        if (bledtm_state == BLEDTM_STATE_TRANSMITTER_TEST)
        {
#if (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)
            // command for proprietary tool (Lab Test Tool).

            if (bledtm_proprietary_physTx_PackageNumberByUser_flag == 1)
            {
                bledtm_proprietary_Tx_UserDefPackageNumber_real += 1;
                if (bledtm_proprietary_Tx_UserDefPackageNumber_real > bledtm_proprietary_Tx_UserDef_PackageNumber_target)
                {
                    return;
                }
            }
            if (bledtm_proprietary_physTx_ChRandom_flag == 1)
            {
                int ch_random_val = 0;

                //channel change
                ch_random_val = (rand() % (BLEDTM_PHYS_CH_MAX - BLEDTM_PHYS_CH_MIN + 1) + BLEDTM_PHYS_CH_MIN);

                //Set RF channel
                BleDTM_channel_set(ch_random_val);
            }
#endif // (ENABLE_LAB_TEST_TOOL_SUPPORTS == ENABLE_DEF)

            //write only header 2 bytes to FIFO in DTM
            BleDTM_txTest_enable(bledtm_air_txPacket.content);
        }
        //---------------------------------------//
    }
}

//260us timer interrupt
void TMR2_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        // flag
        bledtm_timerReport_flag = 1;

    }
}


void BleDTM_Isr(void)
{
    uint16_t rx_packet_len;

    if (BleDTM_intRxState_check() == 1) // RX INT
    {
        bledtm_rxComplete_flag = 1;

        //get CRC_check result
        bledtm_crc_status = BleDTM_crc_checkResult_get();

        if (bledtm_crc_status == 0)
        {
            bledtm_rxComplete_flag = 0;

            // START T/R FSM Manually
            BleDTM_TR_ManualSet();

        }
        else  //CRC =1
        {
            //Read RX FIFO(header, 2Byte)
            setBLE_SpiPDMARxIsr(RX_BUFFER_READ_PORT, (uint32_t)bledtm_air_rx_packet.content, 2);

            if (bledtm_air_rx_packet.content[1] != bledtm_packet_length)
            {
                bledtm_rxComplete_flag = 0;

                // START T/R FSM Manually
                BleDTM_TR_ManualSet();
            }
            else //Len = 37
            {
                rx_packet_len = bledtm_air_rx_packet.content[1] + 3; //crc 3byte

                //Read RX FIFO
                setBLE_SpiPDMARxIsr(RX_BUFFER_READ_PORT, (uint32_t)bledtm_air_rx_packet.content + 2, rx_packet_len);

                rx_packet_len = rx_packet_len + 2;

                if (rx_packet_len >= 3)
                {
                    memcpy(bledtm_pdu.content, bledtm_air_rx_packet.content, rx_packet_len - 3); //only header+payload in m_pdu.content
                }

            }
        } //if(bledtm_crc_status==0)
    }
}
