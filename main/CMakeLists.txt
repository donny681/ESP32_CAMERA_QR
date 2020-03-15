set(COMPONENT_SRCS "app_main.c" "led.c" "qr_recoginize.c")

set(COMPONENT_REQUIRES
    esp32-camera
    nvs_flash
    esp_http_server
    camera
    quirc    
    protocol_examples_common
    )

register_component()
