#ifndef COMPONENTS_OTA_OTA
#define COMPONENTS_OTA_OTA

#include <stdint.h>

uint8_t ota_is_in_progress(void);

void ota_start(void);

void ota_set_url(const char* url);

const char* ota_get_url(void);

const char* ota_get_version(void);

void ota_verify(void);

#endif /* COMPONENTS_OTA_OTA */
