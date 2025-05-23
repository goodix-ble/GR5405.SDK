/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "bat_c.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include <string.h>

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Init bas client.
 *****************************************************************************************
 */
void user_bat_c_init(void);

/**
 *****************************************************************************************
 * @brief Discovery Battery Service on peer.
 *****************************************************************************************
 */
void user_bat_c_disc_srvc_start(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Handle the event for connection.
 *****************************************************************************************
 */
void user_bat_c_connect_handler(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Handle the event for disconnection.
 *****************************************************************************************
 */
void user_bat_c_disconnect_handler(uint8_t conn_idx, uint8_t reason);
