# Header-only: the board's pin map. Every component may depend on this, and it
# depends on nothing, so it cannot introduce a cycle.
idf_component_register(
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio
)
