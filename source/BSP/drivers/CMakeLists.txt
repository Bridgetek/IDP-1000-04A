
add_library(bsp_drivers INTERFACE)

# IDP-1000-04A needs these base device drivers
add_subdirectory(als)
add_subdirectory(efm8_mio)
add_subdirectory(eve_hal)
add_subdirectory(i2c_utils)
add_subdirectory(led)
add_subdirectory(sdcard)
add_subdirectory(tof)

target_link_libraries(bsp_drivers INTERFACE
    als
    efm8_mio
    eve_hal
    i2c_utils
    kinetic_rgb_led
    sdcard
    tof
)
