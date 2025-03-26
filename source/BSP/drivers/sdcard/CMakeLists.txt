add_library(sdcard INTERFACE)

target_include_directories(sdcard INTERFACE  ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(sdcard INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ff.c
    ${CMAKE_CURRENT_LIST_DIR}/ffsystem.c
    ${CMAKE_CURRENT_LIST_DIR}/ffunicode.c
    ${CMAKE_CURRENT_LIST_DIR}/sdcard.c
    ${CMAKE_CURRENT_LIST_DIR}/diskio_rp2040.c
)

target_link_libraries(sdcard INTERFACE pico_stdlib hardware_clocks hardware_spi)