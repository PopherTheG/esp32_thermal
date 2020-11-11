/*
 * eddystone_types.h
 *
 *  Created on: Apr 29, 2020
 *      Author: spestano@spinginemail.com
 */

#ifndef COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_TYPES_H_
#define COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_TYPES_H_

#include <stdint.h>
#include "eddystone_config.h"

typedef uint8_t EddystoneShareSecret_t[16];
typedef uint8_t EddystoneSharedEidKey_t[16];
typedef uint8_t EddystoneAdvData_t[32];
typedef uint8_t EddystoneEidIdentity_t[8];
typedef uint8_t EddystoneChallenge_t[16];
typedef struct {

    EddystoneShareSecret_t shared_secret;

    EddystoneAdvData_t slot_adv_data[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    EddystoneSharedEidKey_t slot_eid_key[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    EddystoneEidIdentity_t slot_eid[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    uint8_t slot_eid_rot_exp[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    int8_t slot_radio_tx_power_level[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    int8_t slot_adv_tx_power_level[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    uint16_t slot_adv_interval[EDDYSTONE_CONFIG_MAX_ADV_SLOT];

    uint8_t eid_used;
}EddystoneParams_t;

#endif /* COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_TYPES_H_ */
