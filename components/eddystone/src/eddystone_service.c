/*
 * eddystone_service.c
 *
 *  Created on: Apr 29, 2020
 *      Author: spestano@spinginemail.com
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "eddystone_platform.h"
#include "eddystone_service.h"
#include "eddystone_types.h"
#include "eddystone_crypto.h"
#include "esp_log.h"

#define EDDYSTONE_LOCK          (uint8_t)0x00
#define EDDYSTONE_UNLOCK        (uint8_t)0x01
#define EDDYSTONE_UNLOCK_AUTO   (uint8_t)0x02

#define ADV_DATA_BROADCAST_OFFSET          10
#define ADV_DATA_SERVICE_DATA_LEN_OFFSET   7

typedef uint8_t EddystoneChallenge_t[16];
typedef uint8_t EddystonePublicECDHKey[32];

typedef struct {
    uint8_t version;
    uint8_t max_supported_total_slots;
    uint8_t max_supported_eid_slots;
    uint8_t capabilities_bit_field;
    uint8_t supported_frame_type_high_bits;
    uint8_t supported_frame_type_low_bits;
    int8_t supported_radio_tx_power[EDDYSTONE_CONFIG_TX_POWER_NUM];
}__attribute__((packed))EddystoneCapabilities_t;

typedef struct{

    /* 128-bit challenge used for authentication */
    EddystoneChallenge_t                challenge;

    /* Beacon parameters */
    EddystoneParams_t                   parameters;

    /* Lock state of the service */
    uint8_t                             lock_state;

    /* Slot to perform read or write */
    uint8_t                             active_slot;

    /* Public ecdh key generated for eid provisioning */
    EddystonePublicECDHKey              public_ecdh_key;

    /* Constant capabilities of beacon */
    EddystoneCapabilities_t             capabilities;

    /* Clock value in seconds used for EID provisioning */
    uint32_t                            clock_val;
}EddystoneService_t;

static EddystoneService_t me;
static const uint8_t eddystone_header[] = { 0x02, 0x01, 0x06, 0x03, 0x03, 0xAA,0xFE};
static const uint8_t default_adv_frame[] = { 0x02, 0x01, 0x06, 0x03, 0x03, 0xAA,0xFE, 0x0d, 0x16, 0xAA,0xFE, 0x10, 0x10, 0x03, 'x', 'e', 'l','e','q','t',0x07};
static uint8_t get_frame_broadcast_len(uint8_t* frame)
{
    if (frame[ADV_DATA_SERVICE_DATA_LEN_OFFSET] > 3) {
        return frame[ADV_DATA_SERVICE_DATA_LEN_OFFSET] - 3;
    }
    return  0;
}

static void set_frame_broadcast_len(uint8_t* frame, uint8_t len)
{
    frame[ADV_DATA_SERVICE_DATA_LEN_OFFSET] = len + 3;
}

static void swap_endian_array(uint8_t* array, uint8_t* big_endian, size_t len)
{
    for (int i = 0; i < len; i++) {
        big_endian[i] = array[len - i - 1];
    }
}

static
void generete_eid(uint8_t *new_eid, uint8_t *eid_key,
        uint8_t rotational_exponent, uint32_t time_sec)
{
    // Calculate the temporary key datastructure 1
    uint8_t ts[4]; // big endian representation of time

    ts[0] = (time_sec >> 24) & 0xFF;
    ts[1] = (time_sec >> 16) & 0xFF;

    uint8_t tmpEidDS1[16] = { 0,0,0,0,0,0,0,0,0,0,0, 0xFF, 0, 0, ts[0], ts[1] };

    // Perform the aes encryption to generate the final temporary key.
    uint8_t tmpKey[16];
    eddystone_crypto_aes_128_encrypt(eid_key, tmpEidDS1, tmpKey);

    // Compute the EID
    uint8_t eid[16];
    uint32_t scaledTime = (time_sec >> rotational_exponent) << rotational_exponent;
    ts[0] = (scaledTime  >> 24) & 0xff;
    ts[1] = (scaledTime >> 16) & 0xff;
    ts[2] = (scaledTime >> 8) & 0xff;
    ts[3] = scaledTime & 0xff;
    uint8_t tmpEidDS2[16] = { 0,0,0,0,0,0,0,0,0,0,0, rotational_exponent, ts[0], ts[1], ts[2], ts[3] };
    eddystone_crypto_aes_128_encrypt(tmpKey, tmpEidDS2, eid);

    // copy the leading 8 bytes of the eid result (full result length = 16) into the ADV frame
    memcpy(new_eid, eid, 8);
}

/**
 * @brief Initialize the eddystone data structure
 * 
 */
void EddystoneService_init(void)
{
    /* Initialize empty structure */
    memset(&me, 0, sizeof(EddystoneService_t));

    /* Set the vtable */

    /* Load saved data fromm persistent storage */
    EddystonePlatform_nvsLoadParams(&me.parameters);

    /* Set the constant configurations */
    me.capabilities.version = EDDYSTONE_CONFIG_VERSION;
    me.capabilities.max_supported_total_slots =
            EDDYSTONE_CONFIG_MAX_ADV_SLOT;
    me.capabilities.max_supported_eid_slots =
            EDDYSTONE_CONFIG_MAX_EID_SLOT;
    me.capabilities.supported_frame_type_low_bits =
            EDDYSTONE_CONFIG_FRAME_SUPPORT_LOWBIT;
    me.capabilities.supported_frame_type_high_bits =
            EDDYSTONE_CONFIG_FRAME_SUPPORT_HIGHBIT;
    me.capabilities.capabilities_bit_field =
            EDDYSTONE_CONFIG_CAPABLIITIES_BIT;
    memcpy(me.capabilities.supported_radio_tx_power,
            supported_radio_tx_power, EDDYSTONE_CONFIG_TX_POWER_NUM);

    int8_t valid_frame_slot = -1;
    /* Make sure there is atleast one valid eddystone frame */
    for (int i = 0; i < EDDYSTONE_CONFIG_MAX_ADV_SLOT; i++) {
        /* Is valid frame */
       if (get_frame_broadcast_len( me.parameters.slot_adv_data[i])){
           valid_frame_slot = i;
           break;
       }
    } 
    /* If there are no valid frames, assign the default URL frame 
     * on the first slot
     */
    if (valid_frame_slot == -1) {
        memcpy(me.parameters.slot_adv_data[0], default_adv_frame, sizeof(default_adv_frame));
    }
    
    /* Set the default advertising interval to 1000 ms */
    for (int i = 0; i < EDDYSTONE_CONFIG_MAX_ADV_SLOT; i++) {
        /* Set only valid adv intervals */
        if ((me.parameters.slot_adv_interval[i] < 100)
                || (me.parameters.slot_adv_interval[i] > 1000)) {

            me.parameters.slot_adv_interval[i] = 1000;
        }
        /* Set the default radio tx power level to the last value in the set */
        me.parameters.slot_radio_tx_power_level[i] =
                me.capabilities.supported_radio_tx_power[EDDYSTONE_CONFIG_TX_POWER_NUM
                        - 1];
    }

}

/**
 * @brief Save written changes to persistent storage.
 * User should call this in the BLE Disconnect event.
 */
void EddystoneService_saveChanges(void)
{
    EddystonePlatform_nvsSaveParams(&me.parameters);
}

/**
 * @brief Copy the advertisement data associated with slot number in slot parameter.
 * 
 * @param adv_data 
 * @param slot 
 */
size_t EddystoneService_advDataGet(uint8_t* adv_data, uint8_t slot)
{
    assert(slot < EDDYSTONE_CONFIG_MAX_ADV_SLOT);
    memcpy(adv_data, me.parameters.slot_adv_data[slot], sizeof(EddystoneAdvData_t));
    return adv_data[7] + 8;
}

/**
 * @brief Deinitialize eddystone structure
 */
void EddystoneService_deinit(void)
{
    memset(&me, 0, sizeof(EddystoneService_t));
}

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
        const uint8_t *data, size_t len)
{
    EddystoneStatus_t status = EDDYSTONE_STATUS_WRITE_NOT_PERMITTED;

    switch(characteristic) {
        case EDDYSTONE_CHAR_ACTIVE_SLOT:
            if (len == sizeof(uint8_t)) {
                if ((me.lock_state == EDDYSTONE_UNLOCK )
                        || (me.lock_state == EDDYSTONE_UNLOCK_AUTO )) {
                    me.active_slot = data[0];
                    status = EDDYSTONE_STATUS_OK;
                }
            } else {
                status = EDDYSTONE_STATUS_INVALID_LENGTH;
            }
            break;
        case EDDYSTONE_CHAR_ADV_INTERVAL:
            if (len == sizeof(uint16_t)) {
                if ((me.lock_state == EDDYSTONE_UNLOCK )
                        || (me.lock_state == EDDYSTONE_UNLOCK_AUTO )) {
                    // Swap BigEndian bytes to little endian
                    me.parameters.slot_adv_interval[me.active_slot] = data[1] << 8;
                    me.parameters.slot_adv_interval[me.active_slot] |= data[0];
                    if ( (me.parameters.slot_adv_interval[me.active_slot] < 100) ||
                            (me.parameters.slot_adv_interval[me.active_slot] > 1000) ) {
                        me.parameters.slot_adv_interval[me.active_slot] = 1000;
                    }
                    status = EDDYSTONE_STATUS_OK;
                }
            } else {
                status = EDDYSTONE_STATUS_INVALID_LENGTH;
            }
            break;
        case EDDYSTONE_CHAR_RADIO_TX_POWER:
        /* TODO: Implement variable variable tx power supported */
            if (me.lock_state == EDDYSTONE_UNLOCK) {
                memset(me.parameters.slot_radio_tx_power_level, data[0], 
                    sizeof(me.parameters.slot_radio_tx_power_level) / me.parameters.slot_radio_tx_power_level[0]);
                status = EDDYSTONE_STATUS_OK;
            }
            break;
        case EDDYSTONE_CHAR_ADV_TX_POWER:
            if (me.lock_state == EDDYSTONE_UNLOCK) {
                me.parameters.slot_adv_tx_power_level[me.active_slot] = data[0];
            }
            break;
        case EDDYSTONE_CHAR_LOCK_STATE:
            if (me.lock_state == EDDYSTONE_UNLOCK) {
                /* Lock request */
                if (len == 1) { 
                    me.lock_state = data[0];
                } else if (len == 17) {
                    me.lock_state = data[0];
                    /* TODO: Decrypt new key and assign it to our current key */
                    
                } else {
                    /* Do nothin */
                }
            }
            break;
        case EDDYSTONE_CHAR_UNLOCK:{
            /* Unlock protocol */
            if (me.lock_state == EDDYSTONE_LOCK) {
                if (len != 16) {
                    status = EDDYSTONE_STATUS_INVALID_LENGTH;
                } else {
                    uint8_t temp[16];

                    /* Encrypt the challenge with our shared secret and compare
                     * it from the value sent by client
                     */
                    eddystone_crypto_aes_128_encrypt(
                            me.parameters.shared_secret, me.challenge, temp);
                    ESP_LOG_BUFFER_HEXDUMP("Secret",me.parameters.shared_secret, sizeof(EddystoneShareSecret_t), ESP_LOG_WARN);
                    ESP_LOG_BUFFER_HEXDUMP("Temp",temp, 16, ESP_LOG_ERROR);
                    ESP_LOG_BUFFER_HEXDUMP("data",data, 16, ESP_LOG_ERROR);
                    if (memcmp(temp, data, sizeof(EddystoneShareSecret_t)) == 0) {
                        me.lock_state = EDDYSTONE_UNLOCK;
                        status = EDDYSTONE_STATUS_OK;
                    }
                }
            }
            break;
        }
        case EDDYSTONE_CHAR_ADV_SLOT_DATA:
            if (me.lock_state == EDDYSTONE_LOCK) {
                break;
            }
            uint8_t frame_type = data[0];
            /*
             * Set the frame type
             */
            me.parameters.slot_adv_data[me.active_slot][ADV_DATA_BROADCAST_OFFSET] =
                    frame_type;

            /* 
             * Set the constant header values of eddystone frames
             */
            memcpy(me.parameters.slot_adv_data[me.active_slot], eddystone_header, sizeof(eddystone_header));

            if (frame_type == 0x30) {
                if (len == 34) {
                    uint8_t pubkey[32], privkey[32];

                    eddystone_crypto_gen_key(pubkey, privkey);

                    eddystone_crypto_gen_ecdh_shared_key(pubkey, privkey,
                            (uint8_t*) &data[1],
                            me.parameters.slot_eid_key[me.active_slot]);

                    generete_eid(me.parameters.slot_eid[me.active_slot],
                            me.parameters.slot_eid_key[me.active_slot],
                            me.parameters.slot_eid_rot_exp[me.active_slot],
                            me.clock_val);

                    swap_endian_array(pubkey, me.public_ecdh_key, 32);

                    /* TODO: Set EID */
                    set_frame_broadcast_len(me.parameters.slot_adv_data[me.active_slot], 10);
                    uint8_t* eid_frame = me.parameters.slot_adv_data[me.active_slot] + ADV_DATA_BROADCAST_OFFSET;
                    eid_frame[0] = frame_type;
                    eid_frame[1] = me.parameters.slot_adv_tx_power_level[me.active_slot];
                    memcpy(eid_frame + 2, me.parameters.slot_eid[me.active_slot], 8);
                    me.parameters.eid_used = 1;

                    status = EDDYSTONE_STATUS_OK;
                } else {
                    /* Other means of eid provisioning
                     * is not supported
                     */
                }
            } else if (frame_type == 0x00 && (len == 17)) {
                /*
                 * Frame type UID
                 * FrameType + Namespace + Instance
                 */


                /*
                 * Set the advertised TX power
                 */
                me.parameters.slot_adv_data[me.active_slot][ADV_DATA_BROADCAST_OFFSET
                        + 1] =
                        me.parameters.slot_adv_tx_power_level[me.active_slot];

                /*
                 * Copy the namespace + instance
                 */
                uint8_t *namespace_instance_ptr =
                        &me.parameters.slot_adv_data[me.active_slot][ADV_DATA_BROADCAST_OFFSET
                                + 2];
                memcpy(namespace_instance_ptr, &data[1], 16);

                set_frame_broadcast_len(me.parameters.slot_adv_data[me.active_slot], 20);
                status = EDDYSTONE_STATUS_OK;
            } else if ((frame_type == 0x10) && (len > 3)) {
                /*
                 * Frame type URL
                 *
                 * Format:
                 * frame_type + url_scheme + url + url_encoding
                 */

                uint8_t *active_adv_data_slot = me.parameters.slot_adv_data[me.active_slot];
                memcpy(active_adv_data_slot + ADV_DATA_BROADCAST_OFFSET + 2, data + 1, len - 1);
                set_frame_broadcast_len(me.parameters.slot_adv_data[me.active_slot], len + 1);
                status = EDDYSTONE_STATUS_OK;
            } else if ((frame_type == 0x20) && (len == 1)) {
                /*
                 * Frame type TLM
                 *
                 * Format:
                 * frame_type only
                 */

                /*
                 * TODO: Handle ETLM if EID is set
                 * otherwise just use unencrypted TLM
                 */

                if (me.parameters.eid_used) {
                    /* TLM Version encrypted */
                    me.parameters.slot_adv_data[me.active_slot][ADV_DATA_BROADCAST_OFFSET + 1] = 1;
                    set_frame_broadcast_len(me.parameters.slot_adv_data[me.active_slot], 18);
                } else {
                    /* TLM Version unencrypted */
                    me.parameters.slot_adv_data[me.active_slot][ADV_DATA_BROADCAST_OFFSET + 1] = 0;
                    set_frame_broadcast_len(me.parameters.slot_adv_data[me.active_slot], 14);
                }

                status = EDDYSTONE_STATUS_OK;
            } else if ((frame_type == 0) && (len == 1)) {
                /* Clear the advertisement slot */
                memset(me.parameters.slot_adv_data[me.active_slot], 0, sizeof(EddystoneAdvData_t));
                status = EDDYSTONE_STATUS_OK;
            }
            break;
        default:
            break;
    }
    return status;
}

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
        uint8_t *buffer, uint16_t *len)
{
    EddystoneStatus_t status = EDDYSTONE_STATUS_READ_NOT_PERMITTED;

    /* Initialize value of length to 0 */
    *len = 0;

    switch (characteritic) {
        case EDDYSTONE_CHAR_CAPABILITIES:
            /* Authorized read only */
            if (me.lock_state != EDDYSTONE_LOCK) {
                memcpy(buffer, &me.capabilities, sizeof(me.capabilities));
                *len = sizeof(me.capabilities);
                status =  EDDYSTONE_STATUS_OK;
            }
            break;
        case EDDYSTONE_CHAR_ACTIVE_SLOT:
            if ((me.lock_state == EDDYSTONE_UNLOCK )
                    || (me.lock_state == EDDYSTONE_UNLOCK_AUTO )) {
                buffer[0] = me.active_slot;
                *len = sizeof(me.active_slot);
                status = EDDYSTONE_STATUS_OK;
            }
            break;
        case EDDYSTONE_CHAR_ADV_INTERVAL:
            if ((me.lock_state == EDDYSTONE_UNLOCK)
                    || (me.lock_state == EDDYSTONE_UNLOCK_AUTO)) {
                /* Swap the bytes to BigEndian */
                buffer[0] = me.parameters.slot_adv_interval[me.active_slot] >> 8;
                buffer[1] = me.parameters.slot_adv_interval[me.active_slot] & 0xFF;
                *len = sizeof(uint16_t);
                status = EDDYSTONE_STATUS_OK;
            }
            break;
        case EDDYSTONE_CHAR_RADIO_TX_POWER:
            if (me.lock_state == EDDYSTONE_LOCK) {
                break;
            }
            buffer[0] = me.parameters.slot_radio_tx_power_level[me.active_slot];
            *len = sizeof(int8_t);
            status = EDDYSTONE_STATUS_OK;
            break;
        case EDDYSTONE_CHAR_LOCK_STATE:{
            /* Allow unauthorized read */
            buffer[0] = me.lock_state;
            *len = sizeof(uint8_t);
            status =  EDDYSTONE_STATUS_OK;
            break;
        }
        case EDDYSTONE_CHAR_UNLOCK:{
            if (me.lock_state == EDDYSTONE_LOCK) {
                /* Copy challenge to local */
                EddystonePlatform_GenerateChallenge(&me.challenge);

                /* Set read lenght */
                *len = sizeof(EddystoneChallenge_t);

                /* Copy challenge to read buffer */
                memcpy(buffer, me.challenge, *len);

                status = EDDYSTONE_STATUS_OK;
            }
            break;
        }
        case EDDYSTONE_CHAR_ADV_SLOT_DATA:{

            if (me.lock_state == EDDYSTONE_LOCK) {
                break;
            }
            size_t broadcast_length = get_frame_broadcast_len(
                    me.parameters.slot_adv_data[me.active_slot]);
            if (me.parameters.slot_adv_data[me.active_slot][ADV_DATA_BROADCAST_OFFSET]
                    == 0x30) {

                /* Eid has a different read format for adv slot */
                buffer[0] = 0x30;
                buffer[1] = me.parameters.slot_eid_rot_exp[me.active_slot];
                buffer[2] = (me.clock_val >> 24) & 0xFF;
                buffer[3] = (me.clock_val >> 16) & 0xFF;
                buffer[4] = (me.clock_val >> 8) & 0xFF;
                buffer[5] = me.clock_val & 0xFF;
                memcpy(buffer + 6,
                        me.parameters.slot_adv_data[me.active_slot]
                                + ADV_DATA_BROADCAST_OFFSET + 2, 8);
                *len = 14;
            } else if (broadcast_length) {
                /* Advertisement slot is not empty */
                memcpy(buffer,
                        me.parameters.slot_adv_data[me.active_slot]
                                + ADV_DATA_BROADCAST_OFFSET, broadcast_length);
                *len = broadcast_length;
            } else {
                /* Empty advertisement slot */
                buffer[0] = 0;
                *len = 1;
            }

            status = EDDYSTONE_STATUS_OK;
            break;
        }
        case EDDYSTONE_CHAR_PUBLIC_ECDH_KEY:
            if (me.lock_state == EDDYSTONE_LOCK) {
                break;
            }

            memcpy(buffer, me.public_ecdh_key, sizeof(me.public_ecdh_key));
            *len = sizeof(me.public_ecdh_key);

            status = EDDYSTONE_STATUS_OK;
            break;
        default:
            break;
    }
    return status;
}
