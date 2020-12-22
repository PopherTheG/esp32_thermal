idf_component_register( SRCS "telemetry_decoder.c" "telemetry_encoder.c" "telemetry.c"
                        INCLUDE_DIRS "."
                        REQUIRES "" 
                        PRIV_REQUIRES "cloud" "thermal" "scanner" "d6t44l")
