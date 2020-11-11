
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "eddystone_platform.h"

#define NVS_PARTITION_NAMESPACE "storage"
#define NVS_KEY_PASSCODE "passcode"
#define NVS_KEY_BLOB "eddystone"

static const char *TAG = "eddystone";

uint32_t EddystonePlatform_getClockValue(void)
{
    return 0;
}

void EddystonePlatform_GenerateChallenge(EddystoneChallenge_t *challenge)
{
    esp_fill_random(*challenge, sizeof(EddystoneChallenge_t));
}

void EddystonePlatform_nvsLoadParams(EddystoneParams_t *data)
{
    nvs_handle_t handle;
    esp_err_t err;

    // Open
    err = nvs_open(NVS_PARTITION_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open error %s", esp_err_to_name(err));
        return;
    }

    size_t len = 0;
    err = nvs_get_blob(handle, NVS_KEY_BLOB, NULL, &len);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS get blob error %s", esp_err_to_name(err));
    }

    if (len != 0)
    {
        len = sizeof(EddystoneParams_t);
        err = nvs_get_blob(handle, NVS_KEY_BLOB, data, &len);
    }
    else
    {
        EddystoneShareSecret_t _def_secret = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
        memcpy(data->shared_secret, _def_secret, sizeof(_def_secret));
        nvs_set_blob(handle, NVS_KEY_BLOB, _def_secret, sizeof(_def_secret));
    }

    nvs_close(handle);
}

void EddystonePlatform_nvsSaveParams(const EddystoneParams_t *data)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(NVS_PARTITION_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open error %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(my_handle, NVS_KEY_BLOB, data, sizeof(EddystoneParams_t));

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS get blob error %s", esp_err_to_name(err));
    }

    nvs_commit(my_handle);

    nvs_close(my_handle);
}
