#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
#define CHAR_MAX_VAL_SIZE           215
#define PROFILE_NUM                 2
#define PREPARE_BUF_MAX_SIZE        1024

enum {
    IDX_SVC,
    IDX_BASIC_CFG_CHAR,
    IDX_BASIC_CFG_VAL,
    IDX_SYS_CFG_CHAR,
    IDX_SYS_CFG_VAL,
    IDX_GPRS_CFG_CHAR,
    IDX_GPRS_CFG_VAL,
    IDX_COMMAND_CHAR,
    IDX_COMMAND_VAL,
    IDX_COMMAND_RSP_CHAR,
    IDX_COMMAND_RSP_VAL,
    IDX_COMMAND_RSP_NTF_CFG,
    IDX_DEVICE_INFO_CHAR,
    IDX_DEVICE_INFO_VAL,
    IDX_DEVICE_INFO_NTF_CFG,
    IDX_EDDYSTONE,
    MAX_CONFIG_IDX,
};


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_notify              = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t command_rsp_ntf_cfg[2]        = {0x00, 0x00};
static const uint8_t device_info_ntf_cfg[2]        = {0x00, 0x00};

static uint8_t configuration_service_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0xc5, 0xe9, 0x7e, 0xab};
static uint8_t configuration_char_basic_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0x01, 0x00, 0x7e, 0xab};
static uint8_t configuration_char_system_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0x02, 0x00, 0x7e, 0xab};

static uint8_t configuration_char_gprs_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0x03, 0x00, 0x7e, 0xab};

static uint8_t command_write_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0x04, 0x00, 0x7e, 0xab};

static uint8_t command_notify_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0x05, 0x00, 0x7e, 0xab};

static uint8_t device_notifications_uuid[] = {
    0xde, 0x90, 0x36, 0x2d, 0x6a, 0x06, 0x07, 0xa1, 0x71,
    0x4e, 0x1e, 0x6e, 0x06, 0x00, 0x7e, 0xab};


static const esp_gatts_attr_db_t gatt_db_configuration[MAX_CONFIG_IDX] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t *)configuration_service_uuid}},

    /* Characteristic Declaration */
    [IDX_BASIC_CFG_CHAR]     =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_BASIC_CFG_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, configuration_char_basic_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_MAX_VAL_SIZE, 0, NULL}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_SYS_CFG_CHAR]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Declaration */
    [IDX_SYS_CFG_VAL]      =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, configuration_char_system_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            CHAR_MAX_VAL_SIZE, 0, NULL}},

    /* Characteristic Value */
    [IDX_GPRS_CFG_CHAR]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Declaration */
    [IDX_GPRS_CFG_VAL]      =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, configuration_char_gprs_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            CHAR_MAX_VAL_SIZE, 0, NULL}},

    /* Command characteristics */
    [IDX_COMMAND_CHAR]      =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_COMMAND_VAL]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, command_write_uuid, ESP_GATT_PERM_WRITE,
            CHAR_MAX_VAL_SIZE, 0, NULL}},

    /* Characteristic Declaration */
    [IDX_COMMAND_RSP_CHAR]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_notify}},

    /* Characteristic Value */
    [IDX_COMMAND_RSP_VAL]  =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, command_notify_uuid, 0,
            CHAR_MAX_VAL_SIZE, 0, NULL}},

    /* Characteristic Value */
    [IDX_COMMAND_RSP_NTF_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(command_rsp_ntf_cfg), (uint8_t *)command_rsp_ntf_cfg}},

    [IDX_DEVICE_INFO_CHAR] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    [IDX_DEVICE_INFO_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)device_notifications_uuid, ESP_GATT_PERM_READ,
            CHAR_MAX_VAL_SIZE, 0, NULL}},

    [IDX_DEVICE_INFO_NTF_CFG] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                sizeof(uint16_t), sizeof(device_info_ntf_cfg), (uint8_t *)device_info_ntf_cfg}},

};


enum
{
    EDDYSTONE_CFG_SVC,
    EDDYSTONE_CHAR_1,
    EDDYSTONE_CHAR_1_VAL,
    EDDYSTONE_CHAR_2,
    EDDYSTONE_CHAR_2_VAL,
    EDDYSTONE_CHAR_3,
    EDDYSTONE_CHAR_3_VAL,
    EDDYSTONE_CHAR_4,
    EDDYSTONE_CHAR_4_VAL,
    EDDYSTONE_CHAR_5,
    EDDYSTONE_CHAR_5_VAL,
    EDDYSTONE_CHAR_6,
    EDDYSTONE_CHAR_6_VAL,
    EDDYSTONE_CHAR_7,
    EDDYSTONE_CHAR_7_VAL,
    EDDYSTONE_CHAR_8,
    EDDYSTONE_CHAR_8_VAL,
    EDDYSTONE_CHAR_9,
    EDDYSTONE_CHAR_9_VAL,
    EDDYSTONE_CHAR_10,
    EDDYSTONE_CHAR_10_VAL,
    EDDYSTONE_CHAR_11,
    EDDYSTONE_CHAR_11_VAL,
    EDDYSTONE_CHAR_12,
    EDDYSTONE_CHAR_12_VAL,
    EDDYSTONE_NB
};

static uint8_t eddystone_svc_uid[]   = {0x95, 0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf, 0x4b, 0xd3, 0x8e, 0x00, 0x75, 0xc8, 0xa3};
static uint8_t eddystone_char1_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x01, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char2_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x02, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char3_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x03, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char4_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x04, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char5_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x05, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char6_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x06, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char7_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x07, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char8_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x08, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char9_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x09, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char10_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x0a, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char11_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x0b, 0x75, 0xc8, 0xa3 };
static uint8_t eddystone_char12_uid[] = { 0x95,  0xe2, 0xed, 0xeb, 0x1b, 0xa0, 0x39, 0x8a, 0xdf,0x4b,0xd3, 0x8e, 0x0c, 0x75, 0xc8, 0xa3 };

static const esp_gatts_attr_db_t eddystone_db[EDDYSTONE_NB] = {
        [EDDYSTONE_CFG_SVC] =
                {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                        ESP_UUID_LEN_128, ESP_UUID_LEN_128, (uint8_t *)eddystone_svc_uid}},
        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_1]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_1_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char1_uid, ESP_GATT_PERM_READ,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_2]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_2_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char2_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_3]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_3_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char3_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_4]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_4_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char4_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_5]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_5_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char5_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_6]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_6_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char6_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_7]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_7_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char7_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_8]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_8_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char8_uid, ESP_GATT_PERM_READ,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_9]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_9_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char9_uid, ESP_GATT_PERM_READ,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_10]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_10_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char10_uid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_11]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_11_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char11_uid,  ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},

        /* Characteristic Declaration */
        [EDDYSTONE_CHAR_12]     =
        {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

        /* Characteristic Value */
        [EDDYSTONE_CHAR_12_VAL] =
        {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (uint8_t *)eddystone_char12_uid,  ESP_GATT_PERM_WRITE,
                CHAR_MAX_VAL_SIZE, 0, NULL}},
};