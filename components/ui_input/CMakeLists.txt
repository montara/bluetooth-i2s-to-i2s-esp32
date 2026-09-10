# lvgl comes in via idf_component.yml; the component manager adds managed
# dependencies to this component's requirements automatically.
idf_component_register(
    SRCS "ui_input.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_pcnt esp_driver_gpio esp_timer
    PRIV_REQUIRES board
)
