/*
 * eddystone_config.h
 *
 *  Created on: Apr 29, 2020
 *      Author: spestano@spinginemail.com
 */

#ifndef COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_CONFIG_H_
#define COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_CONFIG_H_
#include <stdint.h>

#define EDDYSTONE_FRAME_MASK_UID                0x01
#define EDDYSTONE_FRAME_MASK_URL                0x02
#define EDDYSTONE_FRAME_MASK_TLM                0x04
#define EDDYSTONE_FRAME_MASK_EID                0x08
#define EDDYSTONE_FRAME_MASK_ALL                0x0F

#define EDDYSTONE_CONFIG_VERSION                0x00
#define EDDYSTONE_CONFIG_MAX_ADV_SLOT           0x03
#define EDDYSTONE_CONFIG_MAX_EID_SLOT           EDDYSTONE_CONFIG_MAX_ADV_SLOT
#define EDDYSTONE_CONFIG_CAPABLIITIES_BIT       0x00
#define EDDYSTONE_CONFIG_FRAME_SUPPORT_LOWBIT   EDDYSTONE_FRAME_MASK_ALL
#define EDDYSTONE_CONFIG_FRAME_SUPPORT_HIGHBIT  0x00
#define EDDYSTONE_CONFIG_TX_POWER_NUM           5

static const int8_t supported_radio_tx_power[EDDYSTONE_CONFIG_TX_POWER_NUM] = {
        4, 0, -4, -12, -24
};
#endif /* COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_CONFIG_H_ */
