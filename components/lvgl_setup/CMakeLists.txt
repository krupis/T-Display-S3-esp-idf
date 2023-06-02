file(GLOB_RECURSE IMAGE_SOURCES images/*.c)

idf_component_register(SRCS "lvgl_setup.c" "ui_helpers.c" ${IMAGE_SOURCES}
                    INCLUDE_DIRS "include"
                    REQUIRES lvgl esp_lcd esp_timer heap driver)
                    

                    