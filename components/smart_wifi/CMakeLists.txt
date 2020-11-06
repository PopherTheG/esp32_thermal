idf_component_register( SRCS "smart_wifi.c" 
                        INCLUDE_DIRS "."
                        REQUIRES ""
                        PRIV_REQUIRES "nvs_flash" "cloud" "wpa_supplicant")