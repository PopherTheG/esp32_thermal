#ifndef COMPONENTS_OTA_OTA_NVS
#define COMPONENTS_OTA_OTA_NVS
#include <stdlib.h>

void ota_nvs_read_url(char* url, size_t* len);

void ota_nvs_write_url(const char* url, size_t len);
#endif /* COMPONENTS_OTA_OTA_NVS */
