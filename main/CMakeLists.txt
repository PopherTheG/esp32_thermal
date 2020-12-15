set(COMPONENT_REQUIRES "")
set(COMPONENT_PRIV_REQUIRES "" "cloud" "bluetooth" "nvs_flash" "d6t44l" "smart_wifi" "ssd1306" "vl53l3cx" "vl53l1" "scanner" "telemetry" )

set(COMPONENT_SRCS "main.c"  )
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()