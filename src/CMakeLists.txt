set(srcs "esp_hid_device_main.c"
         "esp_hid_gap.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                                    "C:/Users/lorhan/.platformio/packages/framework-espidf/components/bt/include/esp32/include"
                                    "C:/Users/lorhan/.platformio/packages/framework-espidf/components/bt/host/bluedroid/api/include/api"
                       REQUIRES esp_hid bt
                       PRIV_REQUIRES nvs_flash)