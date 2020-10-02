#ifndef COMPONENTS_CLOUD_CLOUD_API
#define COMPONENTS_CLOUD_CLOUD_API

#include <stdint.h>
#include <stdlib.h>

typedef enum {
    CLOUD_RET_UNKNOWN_PROTOCOL = -2,
    CLOUD_RET_ERR = -1,
    CLOUD_RET_OK = 0,
}cloud_ret_t;

typedef enum {
    CLOUD_TYPE_MQTT = 0,
    CLOUD_TYPE_UDP = 1,
    CLOUD_TYPE_TCP = 2,
    CLOUD_TYPE_MQTTS = 3,
}cloud_type_t;

typedef enum {
    CLOUD_EVT_CONNECT,
    CLOUD_EVT_DISCONNECT,
    CLOUD_EVT_RECONNECT,
    CLOUD_EVT_DATA_MQTT,
    CLOUD_EVT_DATA_TCPUDP,
    CLOUD_EVT_OTA_START,
    CLOUD_EVT_OTA_DATA,
    CLOUD_EVT_OTA_END,
}cloud_event_id_t;

typedef struct cloud_event{
    cloud_event_id_t id;
    union {
        union {
            struct {
                const uint8_t* data;
                size_t len;
            }tcp, udp;

            struct {
                const char* topic;
                const uint8_t* data;
                size_t len;
                size_t topic_len;
            }mqtt;
        }data;

        struct {
            const char* bin_name;
            size_t bin_size;
            uint32_t offset;
        }ota_start;

        struct {
            const uint8_t* val;
            size_t len;
        }ota_data;
    }evt;
}cloud_event_t;

typedef void (*cloud_app_cb)(cloud_event_t* evt);

void cloud_api_init(cloud_app_cb app_cb, uint8_t conntype);

void cloud_api_deinit(void);

cloud_ret_t cloud_api_connect_host(const char* dst, uint16_t port);

cloud_ret_t cloud_api_disconnect(void);

cloud_ret_t cloud_api_send(const uint8_t* data, size_t len);

cloud_ret_t cloud_api_set_mqtt_topic(const char* topic, const char* id);

cloud_ret_t cloud_api_set_mqtt_id(const char* id);

const char* cloud_api_get_mqtt_topic(void);

const char* cloud_api_get_mqtt_rx_topic(void);

const char* cloud_api_get_mqtt_tx_topic(void);

cloud_ret_t cloud_api_send_rc_response(const uint8_t* data, size_t len);

#endif /* COMPONENTS_CLOUD_CLOUD_API */
