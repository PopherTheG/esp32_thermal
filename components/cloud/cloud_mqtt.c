
#include <stdlib.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "cloud_mqtt.h"

#define TAG     "mqtt-client"

static esp_mqtt_client_handle_t mqtt_client;

static uint8_t connected = 0;
static cloud_app_cb app_cb = NULL;
static uint8_t disconnect_count = 0;
static const char* rx_topic = NULL;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event) {
    esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id){
        case MQTT_EVENT_CONNECTED: 
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

            connected = 1;
            disconnect_count = 0;
            esp_mqtt_client_subscribe(client, rx_topic, 1);         
            if(app_cb)    {
                cloud_event_t conn_evt = {0};
                conn_evt.id = CLOUD_EVT_CONNECT;
                app_cb(&conn_evt);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT EVENT DISCONNECTED");
            connected = 0;
            disconnect_count++;
            if(app_cb) {
                cloud_event_t conn_evt = {0};
                conn_evt.id = CLOUD_EVT_DISCONNECT;
                app_cb(&conn_evt);
            }

            if(disconnect_count >= 5) {
                disconnect_count = 0;
                if(app_cb) {
                    cloud_event_t conn_evt = {0};
                    conn_evt.id = CLOUD_EVT_RECONNECT;
                    app_cb(&conn_evt);
                }
            }
            break;

        case MQTT_EVENT_SUBSCRIBED:
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA size=%d bytes", event->data_len);
            ESP_LOG_BUFFER_HEXDUMP(TAG, event->data, event->data_len, ESP_LOG_INFO);

            cloud_event_t data_evt = {0};
            data_evt.id = CLOUD_EVT_DATA_MQTT;
            data_evt.evt.data.mqtt.data = (uint8_t *)event->data;
            data_evt.evt.data.mqtt.len = event->data_len;
            data_evt.evt.data.mqtt.topic = event->topic;
            data_evt.evt.data.mqtt.topic_len = event->topic_len;
            
            if (app_cb) {
                (*app_cb)(&data_evt);
            }
            break;
        
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGW(TAG, "Other event id:%d", event->event_id);
            break;
    }

    return ESP_OK;
}

uint8_t cloud_mqtt_init(cloud_app_cb cb) {
    connected = 0;
    app_cb = cb;
    esp_mqtt_client_config_t mqtt_config = {
        .task_prio = 2,
        .transport = MQTT_TRANSPORT_OVER_TCP,
        .event_handle = mqtt_event_handler,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    return mqtt_client != NULL;
}

uint8_t cloud_mqtt_deinit(void) {
    if(mqtt_client) {
        esp_mqtt_client_destroy(mqtt_client);
    }
    return 1;
}

uint8_t cloud_mqtt_connect(const char* dst, uint16_t port) 
{
    rx_topic = cloud_api_get_mqtt_rx_topic();
    if(mqtt_client) {
        esp_mqtt_client_config_t mqtt_config = {
            .host = dst,
            .port = port,
            .task_prio = 2,
            .transport = MQTT_TRANSPORT_OVER_TCP,
            .event_handle = mqtt_event_handler,
        };

        if(esp_mqtt_set_config(mqtt_client, &mqtt_config) == ESP_OK) {
            return esp_mqtt_client_start(mqtt_client) == ESP_OK;
        } else {
            ESP_LOGE(TAG, "Failed to set config");
        }
    }

    return 0;
}

uint8_t cloud_mqtt_disconnect(void) {
    if (mqtt_client) {
        return esp_mqtt_client_stop(mqtt_client) == ESP_OK;
    }
    return 0;
}

uint8_t cloud_mqtt_publish(const char* topic, const uint8_t* data, size_t len, uint8_t qos) {
    if(mqtt_client && connected) {
        ESP_LOGI(TAG, "Publish topic=%s bytes=%d", topic, len);
        if(esp_mqtt_client_publish(mqtt_client, topic, (const char*)data, len, qos, 0) != ESP_FAIL) {
            return 1;
        }
    }
    return 0;
}

uint8_t cloud_mqtt_subscribe(const char* topic)
{
    if(mqtt_client && connected) {
        return esp_mqtt_client_subscribe(mqtt_client, topic, 2) == ESP_OK;
    }

    return 0;
}