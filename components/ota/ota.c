#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "esp_flash_partitions.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_log.h"

#include "ota.h"
#include "ota_nvs.h"
#include "telemetry.h"

#define BUFFSIZE 1024

static const char *TAG = "OTA";

const esp_partition_t *configured = NULL;
const esp_partition_t *running = NULL;

static char url[512] = {0};
static size_t url_len = 0;
static uint8_t ota_started = 0;
static char error_message[1024] = {0};
static int binary_file_length = 0;
static int firmware_size = 0;
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static void task_fatal_error(void) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    abort();
}

static void http_continue_download(esp_http_client_handle_t client, uint32_t start_range) {
    char key_val[128] = {0};
    sprintf(key_val, "bytes=%d-", start_range);

    esp_http_client_set_header(client, "Range", key_val);
    esp_err_t err = esp_http_client_open(client, 0);
    if(err != ESP_OK) {

        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    esp_http_client_fetch_headers(client);
}

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

static void ota_task(void *data)
{
    ota_started = 1;
    (void)data;

    esp_err_t err;
    char *ota_write_data = calloc(1, BUFFSIZE + 1);
    if (ota_write_data == NULL)
    {
        ESP_LOGE(TAG, "Unable to allocate memory for ota write buffer");
        task_fatal_error();
    }

    ota_nvs_read_url(url, &url_len);
    ESP_LOGI(TAG, "len%d URL=%s", url_len, url);

    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x", configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    esp_http_client_config_t config = {
        .url = url,
        .cert_pem = (char *)server_cert_pem_start,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection url=%s", url);
        task_fatal_error();
    }
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    firmware_size = esp_http_client_fetch_headers(client);

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    bool image_header_was_checked = false;

    ESP_LOGI(TAG, "Downloading: %d bytes", firmware_size);
    while (1)
    {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0)
        {
            ESP_LOGE(TAG, "Error: SSL data read error");
            http_cleanup(client);
        }
        else if (data_read > 0)
        {
            if (image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) <= 0) {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            // size_t err_len = sprintf(error_message, "+OTA: Failed new version is lower or the same as invalid version.");
                            // cloud_api_send_rc_response((uint8_t*)error_message, err_len);
                            http_cleanup(client);
                            task_fatal_error();
                        }
                    }

                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) <= 0) {
                        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                        // size_t err_len = sprintf(error_message, "+OTA: Failed new version is lower or the same as current version.");
                        // cloud_api_send_rc_response((uint8_t*)error_message, err_len);
                        http_cleanup(client);
                        task_fatal_error();
                    }

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        task_fatal_error();
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");

                } else {
                    ESP_LOGE(TAG, "received package is not fit len");
                    http_cleanup(client);
                    task_fatal_error();

                }
            }
            err = esp_ota_write(update_handle, (const void*) ota_write_data, data_read);
            if(err != ESP_OK) {
                http_cleanup(client);
                task_fatal_error();
            }
            binary_file_length += data_read;
            ESP_LOGI(TAG, "Download %.4f", ((float) binary_file_length/firmware_size) * 100.0f);            
        } else if(data_read == 0) {
            if(firmware_size > binary_file_length) {
                ESP_LOGW(TAG, "Connection closed, redownload remaining bytes %d", firmware_size - binary_file_length);
                esp_http_client_close(client);
                esp_http_client_cleanup(client);

                client = esp_http_client_init(&config);
                if(client == NULL) {
                    ESP_LOGE(TAG, "Failed to initialise HTTP connection");
                    task_fatal_error();                    
                }                
                http_continue_download(client, binary_file_length);
            } else {
                break;
            }
        }
    }
    ESP_LOGI(TAG, "Total write binary data lenght: %d", binary_file_length);

    if(esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        http_cleanup(client);
        task_fatal_error();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
}

static uint8_t diagnostic(void) {
    return 1;
}

uint8_t ota_is_in_progress(void) {
    return ota_started;
}

void ota_start(void)
{
    ESP_LOGI(TAG, "Started!");
    telemetry_stop();
    telemetry_deinit();

    if (xTaskCreate(ota_task, "ota-task", 8192, NULL, 6, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ota task");
    }
}

void ota_set_url(const char* new_url) {
    strncpy(url, new_url, 255);
    ota_nvs_write_url(url, strlen(url));
}

const char* ota_get_url(void) {
    return url;
}

void ota_verify(void) {
    uint8_t sha_256[32] = {0};
    esp_partition_t partition;

    partition.address   = ESP_PARTITION_TABLE_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type      = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);

    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);

    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if(esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if(ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            uint8_t diagnostic_is_ok = diagnostic();
            if(diagnostic_is_ok) {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();                
            } else {
                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
}

const char* ota_get_version(void)
{
    const esp_app_desc_t *desc = esp_ota_get_app_description();
    return desc->version;
} 