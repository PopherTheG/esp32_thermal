#include "ota_nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define NVS_PARTITION "cfg_part"
#define NVS_PARTITION_NAMESPACE "ota-cfg"
#define NVS_KEY_OTA_URL "url"

void ota_nvs_read_url(char* url, size_t* len) {
    esp_err_t err;
    nvs_handle handle;

    err = nvs_open_from_partition(NVS_PARTITION, NVS_PARTITION_NAMESPACE, NVS_READWRITE, &handle);

    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    err = nvs_get_str(handle, NVS_KEY_OTA_URL, NULL, len);

    err = nvs_get_str(handle, NVS_KEY_OTA_URL, url, len);

    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    nvs_close(handle);

    ESP_LOG_BUFFER_HEXDUMP("OTAURL", url, *len, ESP_LOG_INFO);
}

void ota_nvs_write_url(const char* url, size_t len) {
    esp_err_t err;
    nvs_handle handle;

    err = nvs_open_from_partition(NVS_PARTITION, NVS_PARTITION_NAMESPACE, NVS_READWRITE, &handle);

    ESP_ERROR_CHECK(err);

    err = nvs_set_str(handle, NVS_KEY_OTA_URL, url);

    ESP_ERROR_CHECK(err);

    nvs_close(handle);
}