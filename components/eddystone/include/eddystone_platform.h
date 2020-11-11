/*
 * eddystone_platform.h
 *
 *  Created on: Apr 29, 2020
 *      Author: spestano@spinginemail.com
 *
 */

#ifndef COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_PLATFORM_H_
#define COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_PLATFORM_H_

#include <stdint.h>
#include <stdlib.h>
#include "eddystone_types.h"
/**
 * @brief Get the clock value in seconds
 * @return seconds
 */
uint32_t EddystonePlatform_getClockValue(void);

void EddystonePlatform_GenerateChallenge(EddystoneChallenge_t* challenge);

void EddystonePlatform_nvsLoadParams(EddystoneParams_t* data);

void EddystonePlatform_nvsSaveParams(const EddystoneParams_t* data);

#endif /* COMPONENTS_EDDYSTONE_INCLUDE_EDDYSTONE_PLATFORM_H_ */
