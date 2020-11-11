/*
 * eddystone_service.h
 *
 *  Created on: Apr 29, 2020
 *      Author: spestano@spinginemail.com
 */

#ifndef COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_SERVICE_H_
#define COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_SERVICE_H_

#include <stdlib.h>
#include <stdint.h>

typedef enum {
    EDDYSTONE_STATUS_OK = 0x00,
    EDDYSTONE_STATUS_READ_NOT_PERMITTED = 0x02,
    EDDYSTONE_STATUS_WRITE_NOT_PERMITTED = 0x03,
    EDDYSTONE_STATUS_INVALID_LENGTH = 0x0d,
}EddystoneStatus_t;

typedef enum
{
    EDDYSTONE_CHAR_CAPABILITIES,
    EDDYSTONE_CHAR_ACTIVE_SLOT,
    EDDYSTONE_CHAR_ADV_INTERVAL,
    EDDYSTONE_CHAR_RADIO_TX_POWER,
    EDDYSTONE_CHAR_ADV_TX_POWER,
    EDDYSTONE_CHAR_LOCK_STATE,
    EDDYSTONE_CHAR_UNLOCK,
    EDDYSTONE_CHAR_PUBLIC_ECDH_KEY,
    EDDYSTONE_CHAR_EID_IDENTITY_KEY,
    EDDYSTONE_CHAR_ADV_SLOT_DATA,
    EDDYSTONE_CHAR_FACTORY_RESET,
    EDDYSTONE_CHAR_REMAIN_CONNECTABLE,
}EddystoneCharacteristics_t;


/**
 * @brief Initialize the eddystone data structure
 * 
 */
void EddystoneService_init(void);

/**
 * @brief Deinitialize eddystone structure
 */
void EddystoneService_deinit(void);

/**
 * @brief Save written changes to persistent storage.
 * User should call this in the BLE Disconnect event.
 */
void EddystoneService_saveChanges(void);

/**
 * @brief Copy the advertisement data associated with slot number in slot parameter.
 * 
 * @param adv_data 
 * @param slot 
 */
size_t EddystoneService_advDataGet(uint8_t* adv_data, uint8_t slot);

/**
 * @brief Eddystone Service characteristics write callback.
 * Call this in your BLE write event handler.
 *
 * @param characteristic -  the characteristic to perform the write operation.
 *                          See \rEddystoneCharacteristics_t.
 *
 * @param write_data     -  pointer to the write data
 *
 * @param write_len      -  length of data
 *
 * @return               -  EDDYSTONE_STATUS_OK on success
 *                          EDDYSTONE_STATUS_READ_NOT_PERMITTED on unauthorized read
 *                          EDDYSTONE_STATUS_WRITE_NOT_PERMITTED on unauthorized write
 *                          EDDYSTONE_STATUS_INVALID_LENGTH  on invalid write length
 */
EddystoneStatus_t EddystoneService_writeRequest(EddystoneCharacteristics_t characteristic,
        const uint8_t *data, size_t len);

/**
 * @brief Eddystone Service characteristics read callback.
 * Call this in your BLE read event handler.
 *
 * @param characteritic -   the characteristic to perform the read operation.
 *                          See \rEddystoneCharacteristics_t.
 * @param buffer        -   pointer to store the read data
 *
 * @param len           -   pointer to store the read length
 *
 * @return              -   EDDYSTONE_STATUS_OK on success
 *                          EDDYSTONE_STATUS_READ_NOT_PERMITTED on unauthorized read
 *                          EDDYSTONE_STATUS_WRITE_NOT_PERMITTED on unauthorized write
 *                          EDDYSTONE_STATUS_INVALID_LENGTH  on invalid write length
 */
EddystoneStatus_t EddystoneService_readRequest(EddystoneCharacteristics_t characteritic,
        uint8_t *buffer, uint16_t *len);

#endif /* COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_SERVICE_H_ */
