idf_component_register( SRCS "ota_nvs.c" "ota.c" 
                        INCLUDE_DIRS "."
                        REQUIRES ""
                        PRIV_REQUIRES "nvs_flash" "app_update" "esp_http_client" "smart_wifi" "cloud" "telemetry" "esp_https_ota"
                        EMBED_TXTFILES "certs/ca_cert.pem" )