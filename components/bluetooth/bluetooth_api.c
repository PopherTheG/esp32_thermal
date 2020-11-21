#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "bluetooth_api.h"
#include "eddystone_service.h"
#include "esp_eddystone_api.h"
#include "bluetooth_db.h"

#define TAG "bluetooth"

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE 1024

#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#define GATTS_PROFILE_NUM 2
#define GATTS_PROFILE_A_APP_ID 0
#define GATTS_PROFILE_B_APP_ID 1
#define GATTC_PROFILE_NUM 1
#define GATTC_PROFILE_C_APP_ID 0

// gattc
#define REMOTE_SERVICE_UUID 0x00FF
#define REMOTE_NOTIFY_CHAR_UUID 0xFF01
#define INVALID_HANDLE 0
#define GATTS_ADV_NAME "ESP_GATTS_DEMO"
#define NOTIFY_ENABLE 0x0001
#define INDICATE_ENABLE 0x0002
#define NOTIFY_INDICATE_DISABLE 0x0000
#define CONFIG_SET_RAW_ADV_DATA

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
    uint16_t handle;
} prepare_type_env_t;

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_eddystone_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
/* Declare static functions */
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
static void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static esp_gatt_char_prop_t a_property = 0;
static esp_gatt_char_prop_t b_property = 0;
static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;
static prepare_type_env_t a_prepare_read_env;
static prepare_type_env_t b_prepare_read_env;
static uint8_t adv_config_done = 0;
static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;
static bluetooth_write_handle command_write_handler = NULL;
static uint16_t config_handle_table[MAX_CONFIG_IDX];
static uint16_t eddystone_handle_table[EDDYSTONE_NB];
static bluetooth_scan_cb scan_cb = NULL;
static bluetooth_gatt_cb gatt_user_callback = NULL;
static uint16_t gatts_mtu = 20;
static uint8_t init = 0;

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[31] = {
    /* flags */
    0x02,
    0x01,
    0x06,
    /* tx power*/
    0x02,
    0x0a,
    0xeb,
    /* service uuid */
    0x03,
    0x03,
    0xFF,
    0x00,
};
static size_t raw_adv_len = sizeof(raw_adv_data);

static uint8_t raw_scan_rsp_data[31] = {
    /* flags */
    0x02,
    0x01,
    0x06,
    /* tx power */
    0x02,
    0x0a,
    0xeb,
    /* service uuid */
    0x03,
    0x03,
    0xFF,
    0x00,
    /* device name */
    0x05,
    0x09,
    'A',
    'I',
    'M',
    '_',
};
#else
// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gatts_profile_tab[GATTS_PROFILE_NUM] = {
    [GATTS_PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [GATTS_PROFILE_B_APP_ID] = {
        .gatts_cb = gatts_profile_eddystone_event_handler, /* This demo does not implement, similar as profile A */
        .gatts_if = ESP_GATT_IF_NONE,                      /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = REMOTE_SERVICE_UUID,
    },
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = REMOTE_NOTIFY_CHAR_UUID,
    },
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {
        .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gattc_profile_tab[GATTC_PROFILE_NUM] = {
    [GATTC_PROFILE_C_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGE(TAG, "Advertisement data set complete.");
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done)
        {
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(TAG, "Start advertising...");
        }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "Scan response data set complete.");
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(TAG, "Start advertising..");
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising start failed\n");
        }
        ESP_LOGI(TAG, "Advertising start successfully\n");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising stop failed\n");
        }
        else
        {
            ESP_LOGI(TAG, "Stop adv successfully\n");
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d\n",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "scan stop failed, error status = %x\n", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT, stop scan successfully\n");
        break;

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT, set scan sparameters complete\n");
        //the unit of the duration is second
        uint32_t duration = 0;
        esp_ble_gap_start_scanning(duration);
        break;
    }

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "scan start failed, error status = %x\n", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT, scan start success\n");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {

        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (scan_cb)
            {
                scan_cb((char *)adv_name, adv_name_len, (uint8_t *)scan_result->scan_rst.bda);
            }
            bt_gatt_event_t gattc_event = {0};
            gattc_event.id = BT_GATTC_UKNOWN;
            gattc_event.client.name = (char *)adv_name;
            gattc_event.client.name_len = adv_name_len;
            gattc_event.client.bda = scan_result->scan_rst.bda;
            gattc_event.client.event.uknown_frames.data = scan_result->scan_rst.ble_adv;
            gattc_event.client.event.uknown_frames.len = scan_result->scan_rst.adv_data_len;
            gattc_event.client.rssi = scan_result->scan_rst.rssi;

            /* Check if its eddystone packet */
            esp_eddystone_result_t eddystone_res;
            memset(&eddystone_res, 0, sizeof(eddystone_res));
            esp_err_t ret = esp_eddystone_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, &eddystone_res);
            if (ret)
            {
                // error:The received data is not an eddystone frame packet
                // Check if its iBeacon
                // bluetooth_sensor_process(scan_result->scan_rst.bda, scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
            }
            else
            {
                // The received adv data is a correct eddystone frame packet.
                // Here, we get the eddystone infomation in eddystone_res, we can use the data in res to do other things.
                // For example, just print them:
                gattc_event.id = BT_GATTC_EDDYSTONE;
                if (eddystone_res.common.frame_type == EDDYSTONE_FRAME_TYPE_UID)
                {
                    gattc_event.client.event.eddystone.type = BT_EDDYSTONE_TYPE_UID;
                    gattc_event.client.event.eddystone.frame.uid.ranging_data = eddystone_res.inform.uid.ranging_data;
                    gattc_event.client.event.eddystone.frame.uid.namespace = eddystone_res.inform.uid.namespace_id;
                    gattc_event.client.event.eddystone.frame.uid.instance = eddystone_res.inform.uid.instance_id;
                }
                else if (eddystone_res.common.frame_type == EDDYSTONE_FRAME_TYPE_URL)
                {
                    gattc_event.client.event.eddystone.type = BT_EDDYSTONE_TYPE_URL;
                    gattc_event.client.event.eddystone.frame.url.url = eddystone_res.inform.url.url;
                    gattc_event.client.event.eddystone.frame.url.tx_power = eddystone_res.inform.url.tx_power;
                }
                else
                {
                }
            }

            if (gatt_user_callback)
            {
                gatt_user_callback(&gattc_event);
            }

            static uint8_t is_connected = 0;
            /* This device is configured in autocon, connect to it */
            // if (!is_connected && bluetooth_autocon_check(scan_result->scan_rst.bda, NULL))
            // {
            //     is_connected = 1;
            //     esp_ble_gattc_open(gattc_profile_tab[GATTC_PROFILE_C_APP_ID].gattc_if, scan_result->scan_rst.bda,
            //                        scan_result->scan_rst.ble_addr_type, true);
            // }
            break;

        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT, scan stop\n");
            break;
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

void gatts_proc_read(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_read_env, esp_ble_gatts_cb_param_t *param, uint8_t *p_rsp_v, uint16_t v_len)
{
    if (!param->read.need_rsp)
    {
        return;
    }
    uint16_t value_len = gatts_mtu - 1;
    if (v_len - param->read.offset < (gatts_mtu - 1))
    { // read response will fit in one MTU?
        value_len = v_len - param->read.offset;
    }
    else if (param->read.offset == 0) // it's the start of a long read  (could also use param->read.is_long here?)
    {
        ESP_LOGI(TAG, "long read, handle = %d, value len = %d", param->read.handle, v_len);

        if (v_len > PREPARE_BUF_MAX_SIZE)
        {
            ESP_LOGE(TAG, "long read too long");
            return;
        }
        if (prepare_read_env->prepare_buf != NULL)
        {
            ESP_LOGW(TAG, "long read buffer not free");
            free(prepare_read_env->prepare_buf);
            prepare_read_env->prepare_buf = NULL;
        }

        prepare_read_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_read_env->prepare_len = 0;
        if (prepare_read_env->prepare_buf == NULL)
        {
            ESP_LOGE(TAG, "long read no mem");
            return;
        }
        memcpy(prepare_read_env->prepare_buf, p_rsp_v, v_len);
        prepare_read_env->prepare_len = v_len;
        prepare_read_env->handle = param->read.handle;
    }
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = value_len;
    rsp.attr_value.offset = param->read.offset;
    memcpy(rsp.attr_value.value, &p_rsp_v[param->read.offset], value_len);
    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
}

void gatts_proc_long_read(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_read_env, esp_ble_gatts_cb_param_t *param)
{
    if (prepare_read_env->prepare_buf && (prepare_read_env->handle == param->read.handle)) // something buffered?
    {
        gatts_proc_read(gatts_if, prepare_read_env, param, prepare_read_env->prepare_buf, prepare_read_env->prepare_len);
        if (prepare_read_env->prepare_len - param->read.offset < (gatts_mtu - 1)) // last read?
        {
            free(prepare_read_env->prepare_buf);
            prepare_read_env->prepare_buf = NULL;
            prepare_read_env->prepare_len = 0;
            ESP_LOGI(TAG, "long_read ended");
        }
    }
    else
    {
        ESP_LOGE(TAG, "long_read not buffered");
    }
}

static void gatts_profile_a_write(uint16_t handle, uint8_t *data, size_t len)
{
    bt_gatt_event_t event = {0};
    event.server.write.len = len;
    event.server.write.val = data;
    if (handle == config_handle_table[IDX_COMMAND_VAL])
    {
        event.id = BT_GATTS_CHAR_REMOTE_COMMAND_WRITE;
        if (gatt_user_callback)
        {
            gatt_user_callback(&event);
        }
    }
    else if (handle == config_handle_table[IDX_BASIC_CFG_VAL])
    {
        // bluetooth_config_char
    }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_REG_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db_configuration, gatts_if, MAX_CONFIG_IDX, 0);
        if (create_attr_ret)
        {
            ESP_LOGE(TAG, "create attr table failed, error code = %d", create_attr_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, raw_adv_len);
        if (raw_adv_ret)
        {
            ESP_LOGE(TAG, "config raw adv data failed, error code = %x \n", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(TAG, "config raw scan rsp data failed, error code = %x\n", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#else
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(TAG, "config adv data failed, error code = %x\n", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(TAG, "config scan response data failed, error code = %x\n", ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#endif
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        if (param->read.handle == config_handle_table[IDX_COMMAND_VAL])
        {
        }

        if (!param->read.is_long)
        {
            // gatts_proc_read(gatts_if, &a_prepare_read_env, param, rsp.attr_value.value, rsp.attr_value.len);
            gatts_proc_read(gatts_if, &a_prepare_read_env, param, rsp.attr_value.value, rsp.attr_value.len);
        }
        else
        {
            gatts_proc_long_read(gatts_if, &a_prepare_read_env, param);
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep)
        {
            ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value: ", param->write.len);
            esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            if (gatts_profile_tab[GATTS_PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == NOTIFY_ENABLE)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        ESP_LOGI(TAG, "notify enable\n");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatts_profile_tab[GATTS_PROFILE_A_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                    }
                }
                else if (descr_value == INDICATE_ENABLE)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                    {
                        ESP_LOGI(TAG, "indicate enable\n");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatts_profile_tab[GATTS_PROFILE_A_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == NOTIFY_INDICATE_DISABLE)
                {
                    ESP_LOGI(TAG, "notify/indicate disable \n");
                }
                else
                {
                    ESP_LOGE(TAG, "unknown descr value\n");
                    esp_log_buffer_hex(TAG, param->write.value, param->write.len);
                }
            }
            else
            {
                gatts_profile_a_write(gatts_if, &a_prepare_write_env, param);
            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;

    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT \n");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
        {
            esp_log_buffer_hex(TAG, a_prepare_write_env.prepare_buf, a_prepare_write_env.prepare_len);
            gatts_profile_a_write(a_prepare_write_env.handle, a_prepare_write_env.prepare_buf, a_prepare_write_env.prepare_len);
        }
        else
        {
            ESP_LOGI(TAG, "ESP_GATT_PREP_WRITE_CANCEL\n");
        }
        if (a_prepare_write_env.prepare_buf)
        {
            free(a_prepare_write_env.prepare_buf);
            a_prepare_write_env.prepare_buf = NULL;
        }
        a_prepare_write_env.prepare_len = 0;
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d\n", param->mtu.mtu);
        gatts_mtu = param->mtu.mtu;
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != MAX_CONFIG_IDX)
        {
            ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to MAX_CONFIG_IDX(%d)",
                     param->add_attr_tab.num_handle, MAX_CONFIG_IDX);
        }
        else
        {
            ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(config_handle_table, param->add_attr_tab.handles, sizeof(config_handle_table));
            esp_ble_gatts_start_service(config_handle_table[IDX_SVC]);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:\n",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gatts_profile_tab[GATTS_PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT:

        break;
    default:
        break;
    }
}

static void gatts_profile_eddystone_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(eddystone_db, gatts_if, EDDYSTONE_NB, 1);
        if (create_attr_ret)
        {
            ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
        break;

    case ESP_GATTS_READ_EVT:
    {
        ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

        if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_1_VAL])
        {
            ESP_LOGI(TAG, "Reading eddystone capabilities.");
            EddystoneService_readRequest(EDDYSTONE_CHAR_CAPABILITIES, rsp.attr_value.value, &rsp.attr_value.len);
            ESP_LOG_BUFFER_HEX(TAG, rsp.attr_value.value, rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_2_VAL])
        {
            ESP_LOGI(TAG, "Read active slot");
            EddystoneService_readRequest(EDDYSTONE_CHAR_ACTIVE_SLOT, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_3_VAL])
        {
            ESP_LOGI(TAG, "Read advertisement interval");
            EddystoneService_readRequest(EDDYSTONE_CHAR_ADV_INTERVAL, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_4_VAL])
        {
            ESP_LOGI(TAG, "Read TX power");
            EddystoneService_readRequest(EDDYSTONE_CHAR_RADIO_TX_POWER, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_5_VAL])
        {
            ESP_LOGI(TAG, "Read advertisement TX power");
            EddystoneService_readRequest(EDDYSTONE_CHAR_ADV_TX_POWER, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_6_VAL])
        {
            ESP_LOGI(TAG, "Read lock state");
            EddystoneService_readRequest(EDDYSTONE_CHAR_LOCK_STATE, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_7_VAL])
        {
            ESP_LOGI(TAG, "Read unlock");
            EddystoneService_readRequest(EDDYSTONE_CHAR_UNLOCK, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_8_VAL])
        {
            ESP_LOGI(TAG, "Read ecdh key");
            EddystoneService_readRequest(EDDYSTONE_CHAR_PUBLIC_ECDH_KEY, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_9_VAL])
        {
            ESP_LOGI(TAG, "Read eid identity key");
            EddystoneService_readRequest(EDDYSTONE_CHAR_EID_IDENTITY_KEY, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_10_VAL])
        {
            ESP_LOGI(TAG, "Read slot data");
            EddystoneService_readRequest(EDDYSTONE_CHAR_ADV_SLOT_DATA, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_11_VAL])
        {
            ESP_LOGI(TAG, "Read factory reset");
            EddystoneService_readRequest(EDDYSTONE_CHAR_FACTORY_RESET, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else if (param->read.handle == eddystone_handle_table[EDDYSTONE_CHAR_12_VAL])
        {
            ESP_LOGI(TAG, "Reading capabilities");
            EddystoneService_readRequest(EDDYSTONE_CHAR_REMAIN_CONNECTABLE, rsp.attr_value.value, &rsp.attr_value.len);
        }
        else
        {
            ESP_LOGE(TAG, "Uknown eddystone characteristics %d", param->read.handle);
        }

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);

        break;
    }

    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep)
        {
            ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, value: ", param->write.len);
            esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            if (gatts_profile_tab[GATTS_PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == NOTIFY_ENABLE)
                {
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        ESP_LOGI(TAG, "notify enable\n");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatts_profile_tab[GATTS_PROFILE_B_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                    }
                }
                else if (descr_value == INDICATE_ENABLE)
                {
                    if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                    {
                        ESP_LOGI(TAG, "indicate enable\n");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gatts_profile_tab[GATTS_PROFILE_B_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == NOTIFY_INDICATE_DISABLE)
                {
                    ESP_LOGI(TAG, "notify/indicate disable \n");
                }
                else
                {
                    ESP_LOGE(TAG, "unknown value\n");
                }
            }
            else
            {
                esp_gatt_status_t status = ESP_GATT_OK;
                EddystoneStatus_t eddystone_res = EDDYSTONE_STATUS_WRITE_NOT_PERMITTED;
                /* Eddystone characteristics write */
                if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_1_VAL])
                {
                    ESP_LOGI(TAG, "Write eddystone capabilities %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_CAPABILITIES, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_2_VAL])
                {
                    ESP_LOGI(TAG, "Write active slot %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ACTIVE_SLOT, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_3_VAL])
                {
                    ESP_LOGI(TAG, "Write advertisement interval %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ADV_INTERVAL, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_4_VAL])
                {
                    ESP_LOGI(TAG, "Write radio TX power %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_RADIO_TX_POWER, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_5_VAL])
                {
                    ESP_LOGI(TAG, "Write advertisement tx power %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ADV_TX_POWER, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_6_VAL])
                {
                    ESP_LOGI(TAG, "Write lock state %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_LOCK_STATE, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_7_VAL])
                {
                    ESP_LOGI(TAG, "Write unlock state %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_UNLOCK, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_8_VAL])
                {
                    ESP_LOGI(TAG, "Write ecdh key %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_PUBLIC_ECDH_KEY, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_9_VAL])
                {
                    ESP_LOGI(TAG, "Write eid %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_EID_IDENTITY_KEY, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_10_VAL])
                {
                    ESP_LOGI(TAG, "Write adv slot %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ADV_SLOT_DATA, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_11_VAL])
                {
                    ESP_LOGI(TAG, "Write factory reset %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_FACTORY_RESET, param->write.value, param->write.len);
                }
                else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_12_VAL])
                {
                    ESP_LOGI(TAG, "Write remain connected %d", param->write.handle);
                    eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_REMAIN_CONNECTABLE, param->write.value, param->write.len);
                }
                status = eddystone_res;
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
            }
        }
        example_write_event_env(gatts_if, &b_prepare_write_env, param);
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&b_prepare_write_env, param);
        break;

    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d\n", param->mtu.mtu);
        gatts_mtu = param->mtu.mtu;
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != EDDYSTONE_NB)
        {
            ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) doesn't equal to MAX_CONFIG_IDX(%d)", param->add_attr_tab.num_handle, EDDYSTONE_NB);
        }
        else
        {
            ESP_LOGI(TAG, "create attribe table succesfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(eddystone_handle_table, param->add_attr_tab.handles, sizeof(eddystone_handle_table));
            esp_ble_gatts_start_service(eddystone_handle_table[EDDYSTONE_CFG_SVC]);
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x\n",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gatts_profile_tab[GATTS_PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT status %d attr_handle %d\n", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            esp_log_buffer_hex(TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_DISCONNECT_EVT:
    case ESP_GATTS_OPEN_EVT:
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gattc_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(TAG, "reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    
    do
    {
        int idx;
        for (idx = 0; idx < GATTC_PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gattc_if == gattc_profile_tab[idx].gattc_if)
            {
                if (gattc_profile_tab[idx].gattc_cb)
                {
                    gattc_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gatts_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(TAG, "Reg app failed, app_id %04x, status %d\n",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < GATTS_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gatts_profile_tab[idx].gatts_if)
            {
                if (gatts_profile_tab[idx].gatts_cb)
                {
                    gatts_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(TAG, "REG_EVT\n");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret)
        {
            ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d\n", p_data->connect.conn_id, gattc_if);
        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gattc_profile_tab[GATTC_PROFILE_C_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "REMOTE BDA:");
        esp_log_buffer_hex(TAG, gattc_profile_tab[GATTC_PROFILE_C_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->connect.conn_id);
        if (mtu_ret)
        {
            ESP_LOGE(TAG, "config MTU error, error code = %x\n", mtu_ret);
        }
        break;

    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "open failed, status %d\n", p_data->open.status);
            break;
        }
        ESP_LOGI(TAG, "open success\n");
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "discover service failed, status %d\n", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "discover service complete conn_id %d\n", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "config mtu failed, error status = %x\n", param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d\n", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(TAG, "SEARCH RES: conn_id = %x is primary service %d\n", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(TAG, "start handle %d end handle %d current handle value %d\n", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID)
        {
            ESP_LOGI(TAG, "service found\n");
            get_server = true;
            gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_end_handle = p_data->search_res.end_handle;
            ESP_LOGI(TAG, "UUID16: %x\n", p_data->search_res.srvc_id.uuid.uuid.uuid16);
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "search service failed, error status = %x\n", p_data->search_cmpl.status);
            break;
        }
        if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE)
        {
            ESP_LOGI(TAG, "Get service information from remote device\n");
        }
        else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH)
        {
            ESP_LOGI(TAG, "Get service information from flash\n");
        }
        else
        {
            ESP_LOGI(TAG, "unknown service source\n");
        }
        ESP_LOGI(TAG, "ESP_GATTC_SEARCH_CMPL_EVT\n");
        if (get_server)
        {
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_start_handle,
                                                                    gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            if (status != ESP_GATT_OK)
            {
                ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error\n");
            }

            if (count > 0)
            {
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result)
                {
                    ESP_LOGE(TAG, "gattc no mem\n");
                }
                else
                {
                    status = esp_ble_gattc_get_char_by_uuid(gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_start_handle,
                                                            gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK)
                    {
                        ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid error\n");
                    }

                    /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].char_handle = char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if, gattc_profile_tab[GATTC_PROFILE_C_APP_ID].remote_bda, char_elem_result[0].char_handle);
                    }
                }
                /* free char_elem_result */
                free(char_elem_result);
            }
            else
            {
                ESP_LOGE(TAG, "no char found\n");
            }
        }
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        ESP_LOGI(TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT\n");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "REG FOR NOTIFY failed: error status = %d\n", p_data->reg_for_notify.status);
        }
        else
        {
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_start_handle,
                                                                        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].service_end_handle,
                                                                        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].char_handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK)
            {
                ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error\n");
            }
            if (count > 0)
            {
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result)
                {
                    ESP_LOGE(TAG, "malloc error, gattc no mem\n");
                }
                else
                {
                    ret_status = esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                                                        gattc_profile_tab[GATTC_PROFILE_C_APP_ID].conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(TAG, "esp_ble_gattc_get_descr_by_char_handle error\n");
                    }
                    /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                    if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                    {
                        ret_status = esp_ble_gattc_write_char_descr(gattc_if,
                                                                    gattc_profile_tab[GATTC_PROFILE_C_APP_ID].conn_id,
                                                                    descr_elem_result[0].handle,
                                                                    sizeof(notify_en),
                                                                    (uint8_t *)&notify_en,
                                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                                    ESP_GATT_AUTH_REQ_NONE);
                    }

                    if (ret_status != ESP_GATT_OK)
                    {
                        ESP_LOGE(TAG, "esp_ble_gattc_write_char_descr error\n");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else
            {
                ESP_LOGE(TAG, "decsr not found\n");
            }
        }
        break;

    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
        {
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        }
        else
        {
            ESP_LOGI(TAG, "ESP_GATTC_NOTIFY_EVT, receive indicate value:");
        }
        esp_log_buffer_hex(TAG, p_data->notify.value, p_data->notify.value_len);
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "write descr failed, error status = %x\n", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write descr success \n");
        uint8_t write_char_data[35];
        for (int i = 0; i < sizeof(write_char_data); ++i)
        {
            write_char_data[i] = i % 256;
        }
        esp_ble_gattc_write_char(gattc_if,
                                 gattc_profile_tab[GATTC_PROFILE_C_APP_ID].conn_id,
                                 gattc_profile_tab[GATTC_PROFILE_C_APP_ID].char_handle,
                                 sizeof(write_char_data),
                                 write_char_data,
                                 ESP_GATT_WRITE_TYPE_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
    {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "write char failed, error status = %x\n", p_data->write.status);
            break;
        }
        ESP_LOGI(TAG, "write char success \n");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d\n", p_data->disconnect.reason);
        break;

    default:
        break;
    }
}

static void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_buf = 0;
                prepare_write_env->handle = param->write.handle;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    ESP_LOGE(TAG, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }
            else
            {
                if (param->write.offset > PREPARE_BUF_MAX_SIZE)
                {
                    status = ESP_GATT_INVALID_OFFSET;
                }
                else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
                {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(TAG, "Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset, param->write.value, param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            EddystoneStatus_t eddystone_res = EDDYSTONE_STATUS_WRITE_NOT_PERMITTED;
            /* Eddystone characteristics write */
            if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_1_VAL])
            {
                ESP_LOGI(TAG, "Write eddystone capabilities %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_CAPABILITIES, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_2_VAL])
            {
                ESP_LOGI(TAG, "Write active slot %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ACTIVE_SLOT, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_3_VAL])
            {
                ESP_LOGI(TAG, "Write advertisement interval %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ADV_INTERVAL, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_4_VAL])
            {
                ESP_LOGI(TAG, "Write radio TX power %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_RADIO_TX_POWER, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_5_VAL])
            {
                ESP_LOGI(TAG, "Write advertisement tx power %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ADV_TX_POWER, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_6_VAL])
            {
                ESP_LOGI(TAG, "Write lock state %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_LOCK_STATE, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_7_VAL])
            {
                ESP_LOGI(TAG, "Write unlock state %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_UNLOCK, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_8_VAL])
            {
                ESP_LOGI(TAG, "Write ecdh key %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_PUBLIC_ECDH_KEY, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_9_VAL])
            {
                ESP_LOGI(TAG, "Write eid %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_EID_IDENTITY_KEY, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_10_VAL])
            {
                ESP_LOGI(TAG, "Write adv slot %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_ADV_SLOT_DATA, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_11_VAL])
            {
                ESP_LOGI(TAG, "Write factory reset %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_FACTORY_RESET, param->write.value, param->write.len);
            }
            else if (param->write.handle == eddystone_handle_table[EDDYSTONE_CHAR_12_VAL])
            {
                ESP_LOGI(TAG, "Write remain connected %d", param->write.handle);
                eddystone_res = EddystoneService_writeRequest(EDDYSTONE_CHAR_REMAIN_CONNECTABLE, param->write.value, param->write.len);
            }
            status = eddystone_res;
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

static void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGE(TAG, "ESP_GATT_PREP_WRITE_CANCEL\n");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

void bluetooth_init(char *name)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Setting bluetooth name %s", name);
    strcpy((char *)raw_scan_rsp_data + 12, name);
    raw_scan_rsp_data[10] = strlen(name) + 1;

    EddystoneService_init();

    raw_adv_len = EddystoneService_advDataGet(raw_adv_data, 0);
    ESP_LOG_BUFFER_HEXDUMP(TAG, raw_adv_data, raw_adv_len, ESP_LOG_INFO);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_profile_a_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(GATTS_PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(GATTS_PROFILE_B_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    // gattc regisrter
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(TAG, "%s gattc register failed, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(GATTC_PROFILE_C_APP_ID);
    if (ret)
    {
        ESP_LOGE(TAG, "%s gattc app register failed, error code = %x\n", __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    init = 1;
}

void bluetooth_deinit(void)
{
    init = 0;
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
}

void bluetooth_register_gatt_handler(bluetooth_gatt_cb gatt_handler)
{
    gatt_user_callback = gatt_handler;
}

void bluetooth_register_command_write_handler(bluetooth_write_handle write_handler)
{
    command_write_handler = write_handler;
}

void bluetooth_command_response_notify(const char *data, size_t len)
{
    if (init)
    {

        esp_err_t err = esp_ble_gatts_send_indicate(gatts_profile_tab[0].gatts_if, gatts_profile_tab[0].conn_id,
                                                    config_handle_table[IDX_COMMAND_RSP_VAL], len, (uint8_t *)data, 0);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }
}

void bluetooth_device_notification(const char *data, size_t len)
{
    if (init)
    {
        esp_ble_gatts_send_indicate(gatts_profile_tab[0].gatts_if, gatts_profile_tab[0].conn_id,
                                    config_handle_table[IDX_DEVICE_INFO_VAL], len, (uint8_t *)data, 0);
    }
}

void bluetooth_scan_start(bluetooth_scan_cb scan_callback)
{
    scan_cb = scan_callback;
    esp_ble_gap_start_scanning(5);
}

void bluetooth_client_write(const char *data, size_t len)
{
    esp_ble_gattc_write_char(gattc_profile_tab[GATTC_PROFILE_C_APP_ID].gattc_if,
                             gattc_profile_tab[GATTC_PROFILE_C_APP_ID].conn_id,
                             gattc_profile_tab[GATTC_PROFILE_C_APP_ID].char_handle,
                             len,
                             data,
                             ESP_GATT_WRITE_TYPE_RSP,
                             ESP_GATT_AUTH_REQ_NONE);
}