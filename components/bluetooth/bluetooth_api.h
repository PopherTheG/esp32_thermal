#ifndef COMPONENTS_BLUETOOTH_BLUETOOTH_API
#define COMPONENTS_BLUETOOTH_BLUETOOTH_API

#include <stdint.h>

#define BT_EDDYSTONE_NAMESPACE_LEN 10
#define BT_EDDYSTONE_INSTANCE_LEN 6
#define BT_BDA_LEN 6

typedef enum {
    BT_GATTS_CHAR_UNKOWN,
    BT_GATTS_CHAR_REMOTE_COMMAND_WRITE,
    BT_GATTC_EDDYSTONE,
    BT_GATTC_IBEACON,
    BT_GATTC_UKNOWN,
}bt_gatt_event_id_t;

enum {
    BT_EDDYSTONE_TYPE_UID,
    BT_EDDYSTONE_TYPE_URL,
    BT_EDDYSTONE_TYPE_TLM,
    BT_EDDYSTONE_TYPE_EID,
};

typedef struct bt_gatt_event {
    bt_gatt_event_id_t id;

    union {
        struct {
            uint8_t* buff;
            size_t len;
        }read;
        struct {
            uint8_t* val;
            size_t len;
        }write;
    }server;
    struct {                
        char* name;
        uint8_t name_len;
        uint8_t* bda;       /* Pointer to client mac address (6 byte array) */
        int8_t rssi;

        union {
            struct {
                uint8_t type;
                union {
                    struct {
                        int8_t ranging_data;
                        uint8_t *namespace; /* Pointer to namespace (10 - bytes) */
                        uint8_t *instance;  /* Pointer to instance id (6 - bytes) */
                    } uid;

                    struct {
                        int8_t tx_power;
                        char *url;
                    } url;
                } frame;
            } eddystone;

            struct {
                int8_t ranging_data;
                uint8_t* uuid;
                uint8_t minor;
                uint8_t major;
            }ibeacon;

            struct {
                uint8_t* data;
                size_t len;
            }uknown_frames;
        }event;
    }client;

}bt_gatt_event_t;

typedef void (*bluetooth_gatt_cb)(bt_gatt_event_t* event);
typedef void (*bluetooth_write_handle)(const uint8_t* data, size_t len);
typedef void (*bluetooth_scan_cb)(const char* name,  size_t name_len, const uint8_t* bd_addr);

/**
 * @brief Initialize bluetooth stack
 * 
 * @param name 
 */
void bluetooth_init(char* name);

/**
 * @brief Freeup bluetooth stack
 * 
 */
void bluetooth_deinit(void);

/**
 * @brief Register a user application callback for bluetooth events
 * 
 * @param gatts_handler 
 */
void bluetooth_register_gatt_handler(bluetooth_gatt_cb gatts_handler);

/**
 * 
 * @brief Obsolete TODO: Delete this!
 * 
 * @param write_handler 
 */
void bluetooth_register_command_write_handler(bluetooth_write_handle write_handler);

/**
 * @brief Send data to command response notification characteristic
 * 
 * @param data 
 * @param len 
 */
void bluetooth_command_response_notify(const char* data, size_t len);

/**
 * @brief Send data to the device info notification characteristic
 * 
 * @param data 
 * @param len 
 */
void bluetooth_device_notification(const char* data, size_t len);

/**
 * @brief Start bluetooth scan.
 * 
 * @param scan_cb callback for scan results
 * 
 */
void bluetooth_scan_start(bluetooth_scan_cb scan_cb);

/**
 * @brief Write data to the client.
 * 
 * @param data 
 * @param len 
 */
void bluetooth_client_write(const char* data, size_t len);

#endif /* COMPONENTS_BLUETOOTH_BLUETOOTH_API */
