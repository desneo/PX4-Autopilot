px4_add_module(
    MODULE examples__serial_probe
    MAIN serial_probe
    SRCS serial_probe.cpp
    MODULE_CONFIG
        module.yaml
)
