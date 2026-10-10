px4_add_module(
    MODULE examples__serial_probe
    MAIN serial_probe
    SRCS serial_probe.cpp
    DEPENDS
        px4_work_queue
)
