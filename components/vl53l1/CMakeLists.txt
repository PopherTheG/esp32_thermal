idf_component_register( SRCS "vl53l1.c"                            
                            "core/VL53L1X_api.c"
                            "core/VL53L1X_calibration.c"
                            "platform/vl53l1_platform.c"
                        INCLUDE_DIRS "core" "platform" "."
                        REQUIRES "" )