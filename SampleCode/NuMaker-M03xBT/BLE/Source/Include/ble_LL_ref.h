/*----------------------------------------------------------------------------*/
/* This file defined Message Block Related */
/*----------------------------------------------------------------------------*/
#include <stdint.h>
#include "ble_msgblock.h"

/**************************************************************************/
/**
 * @attention The definitions in this file shall NOT be modified.
 * @note The definitions in this file are defined for link layer.
 **************************************************************************/

/** REF_SIZE_LL_CONN for checking connection parameter size in link layer. */
#define REF_SIZE_LL_CONN            144     //LL_Conn

/** REF_SIZE_MQUEUE for checking queue size in link layer. */
#define REF_SIZE_MQUEUE             8       //MQUEUE

/** REF_SIZE_LL_LENGTH_PREFER for checking the packet length exchange information size in link layer. */
#define REF_SIZE_LL_LENGTH_PREFER   8       //LL_Length_Prefer

/** REF_SIZE_TMRBLK_LL for checking the timing mechanism size in link layer. */
#define REF_SIZE_TMRBLK_LL          8       //TBLK_LLx

/** REF_SIZE_CMDTMR for checking the BLE host timers size in host layer. */
#define REF_SIZE_CMDTMR             60

/** NUM_LL_ADV_SCN_INIT_USR is the number representing BLE link layer adverting, scan, initialing and user defined state.
  * (1+1+1) = (Scan + initialing + user data)
  */
#define NUM_LL_ADV_SCN_INIT_USR     (1 + 1 + 1)


#if (NUM_LL_LINK != 0)
/* Extern the predefined and reserved block of memory used as an image for the BLE link layer control parameters. */
extern uint32_t param_rsv_LL_Conn[][(REF_SIZE_LL_CONN >> 2)];

/* Extern the predefined and reserved block of memory used as an image for the BLE link layer data packet queue. */
extern uint32_t param_rsv_LL_ConnDataQ[][(REF_SIZE_MQUEUE >> 2)];

/* Extern the predefined and reserved block of memory used as an image for the BLE link layer control packet queue. */
extern uint32_t param_rsv_LL_ConnCtrlQ[][(REF_SIZE_MQUEUE >> 2)];

/* Extern the predefined and reserved block of memory used as an image for the BLE link layer received data packet queue. */
extern uint32_t param_rsv_LL_ConnDataInQ[][(REF_SIZE_MQUEUE >> 2)];

/* Extern the predefined and reserved block of memory used as an image for the BLE link layer large length data packet buffer pointers. */
extern uint32_t *param_rsv_LL_ConnBuffPt[];

/* Extern the predefined and reserved block of memory used as an image for the BLE link layer temporary parameters or events pointers. */
extern uint32_t *param_rsv_mblk_LL_conn_Para[];

/* Extern the definition of the preferred BLE link layer packet length exchange information. */
extern uint16_t LL_Length_Prefer[][(REF_SIZE_LL_LENGTH_PREFER >> 1)];   //"/2":uint16_t -> uint8_t

/* Extern the predefined and reserved block of memory used for the BLE host timers. */
extern uint32_t param_rsv_host_timer[][REF_SIZE_CMDTMR >> 2];

#endif // (#if (NUM_LL_LINK != 0))

/* Extern the predefined and reserved block of memory used as an image for the BLE link layer timing mechanism. */
extern uint32_t param_rsv_TmrBlk_LL[][(REF_SIZE_TMRBLK_LL >> 2)];

/* Extern the predefined number to represent the number of the BLE link layer timing mechanism units. */
extern const uint8_t MAX_TBLK_LL_NO;

/* Extern the predefined number to represent the number of BLE link layer master and slave roles.*/
extern const uint8_t MAX_NUM_CONN_HDL;


