#ifndef COMPONENTS_CLOUD_CLOUD_CONFIG_DEFAULT
#define COMPONENTS_CLOUD_CLOUD_CONFIG_DEFAULT

#ifndef CLOUD_CFG_MQTT_TOPIC_MAIN
#define CLOUD_CFG_MQTT_TOPIC_MAIN               "xeleqt/aim1"
#endif

#ifndef CLOUD_CFG_MQTT_TOPIC_UI
#define CLOUD_CFG_MQTT_TOPIC_UI                 "xeleqt/aim1/ui"
#endif

#ifndef CLOUD_CFG_MQTT_BASE_TOPIC_RC_RX
#define CLOUD_CFG_MQTT_BASE_TOPIC_RC_RX         "xeleqt/aim1/%s/cmd"
#endif

#ifndef CLOUD_CFG_MQTT_BASE_TOPIC_RC_TX
#define CLOUD_CFG_MQTT_BASE_TOPIC_RC_TX         "xeleqt/aim1/%s/rsp"
#endif

#ifndef CLOUD_CFG_MQTT_BASE_TOPIC_OTA_DATA      
#define CLOUD_CFG_MQTT_BASE_TOPIC_OTA_DATA      "xeleqt/aim1/%s/ota_data"
#endif

#endif /* COMPONENTS_CLOUD_CLOUD_CONFIG_DEFAULT */
