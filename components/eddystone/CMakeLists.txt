idf_component_register(SRCS "src/eddystone_crypto.c" "src/eddystone_platform.c"
                        "src/eddystone_service.c" "src/esp_eddystone_api.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "bt"
                       PRIV_REQUIRES "nvs_flash")
