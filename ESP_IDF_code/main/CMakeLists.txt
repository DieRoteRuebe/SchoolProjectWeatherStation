idf_component_register(SRCS "weather_station_project.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver
                    REQUIRES esp_wifi
                    REQUIRES nvs_flash
                    REQUIRES mqtt)
