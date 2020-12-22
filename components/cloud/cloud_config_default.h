#ifndef COMPONENTS_CLOUD_CLOUD_CONFIG_DEFAULT
#define COMPONENTS_CLOUD_CLOUD_CONFIG_DEFAULT

#ifndef CLOUD_CFG_MQTT_HOST
#define CLOUD_CFG_MQTT_HOST                     "52.221.96.155"
#endif

#ifndef CLOUD_CFG_MQTT_PORT
#define CLOUD_CFG_MQTT_PORT                     2883
#endif

#ifndef CLOUD_CFG_MQTT_TOPIC_MAIN
#define CLOUD_CFG_MQTT_TOPIC_MAIN               "xeleqt/v1"
#endif

#ifndef CLOUD_CFG_MQTT_TOPIC_UI
#define CLOUD_CFG_MQTT_TOPIC_UI                 "xeleqt/v1/ui"
#endif

#ifndef CLOUD_CFG_MQTT_BASE_TOPIC_RC_RX
#define CLOUD_CFG_MQTT_BASE_TOPIC_RC_RX         "xeleqt/v1/%s"
#endif

#ifndef CLOUD_CFG_MQTT_BASE_TOPIC_RC_TX
#define CLOUD_CFG_MQTT_BASE_TOPIC_RC_TX         "xeleqt/v1/%s/rsp"
#endif

#ifndef CLOUD_CFG_MQTT_BASE_TOPIC_OTA_DATA      
#define CLOUD_CFG_MQTT_BASE_TOPIC_OTA_DATA      "xeleqt/v1/%s/ota_data"
#endif

#endif /* COMPONENTS_CLOUD_CLOUD_CONFIG_DEFAULT */
