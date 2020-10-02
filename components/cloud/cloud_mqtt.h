#ifndef COMPONENTS_CLOUD_CLOUD_MQTT
#define COMPONENTS_CLOUD_CLOUD_MQTT

#include <stdint.h>
#include <stdlib.h>
#include "cloud_api.h"

uint8_t cloud_mqtt_init(cloud_app_cb cb);

uint8_t cloud_mqtt_deinit(void);

uint8_t cloud_mqtt_connect(const char* dst, uint16_t port);

uint8_t cloud_mqtt_disconnect(void);

uint8_t cloud_mqtt_publish(const char* topic, const uint8_t* data, size_t len, uint8_t qos);

uint8_t cloud_mqtt_subscribe(const char* topic);

#endif /* COMPONENTS_CLOUD_CLOUD_MQTT */
