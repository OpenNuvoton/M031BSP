/*----------------------------------------------------------------------------*/
/* This file defined Link Layer Related */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "ble_msgblock.h"
#include "ble_LL_ref.h"

/**************************************************************************/
/**
 * @attention The definitions in this file shall NOT be modified.
 * @note The definitions in this file are defined for link layer.
 **************************************************************************/

#if (NUM_LL_LINK != 0)

/* A predefined and reserved block of memory used as an image for the BLE link layer control parameters. */
uint32_t param_rsv_LL_Conn[NUM_LL_LINK][(REF_SIZE_LL_CONN >> 2)];

/* A predefined and reserved block of memory used as an image for the BLE link layer data packet queue. */
uint32_t param_rsv_LL_ConnDataQ[NUM_LL_LINK][(REF_SIZE_MQUEUE >> 2)];

/* A predefined and reserved block of memory used as an image for the BLE link layer control packet queue. */
uint32_t param_rsv_LL_ConnCtrlQ[NUM_LL_LINK][(REF_SIZE_MQUEUE >> 2)];

/* A predefined and reserved block of memory used as an image for the BLE link layer received data packet queue. */
uint32_t param_rsv_LL_ConnDataInQ[NUM_LL_LINK][(REF_SIZE_MQUEUE >> 2)];

/* A predefined and reserved block of memory used as an image for the BLE link layer large length data packet buffer pointers. */
uint32_t *param_rsv_LL_ConnBuffPt[NUM_LL_LINK];

/* A predefined and reserved block of memory used as an image for the BLE link layer temporary parameters or events pointers. */
uint32_t *param_rsv_mblk_LL_conn_Para[NUM_LL_LINK];

/* Used for the BLE link layer packet length exchange information by the application. */
uint16_t LL_Length_Prefer[NUM_LL_LINK][(REF_SIZE_LL_LENGTH_PREFER >> 1)];

/* A predefined and reserved block of memory used for the BLE host timers. */
uint32_t param_rsv_host_timer[NUM_LL_LINK][REF_SIZE_CMDTMR >> 2];

#endif // (#if (NUM_LL_LINK != 0))


/* A predefined and reserved block of memory used as an image for the BLE link layer timing mechanism. */
uint32_t param_rsv_TmrBlk_LL[(2 * (NUM_LL_LINK + NUM_LL_ADV_SCN_INIT_USR))][(REF_SIZE_TMRBLK_LL >> 2)];

/* A predefined number to represent the number of the BLE link layer timing mechanism units. */
const uint8_t MAX_TBLK_LL_NO = (2 * (NUM_LL_LINK + NUM_LL_ADV_SCN_INIT_USR));

/* A predefined number to represent the number of BLE link layer master and slave roles.*/
const uint8_t MAX_NUM_CONN_HDL = NUM_LL_LINK;

