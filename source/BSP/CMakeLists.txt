cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD "bridgetek_idp_1000_04a")

# Initialize the SDK
pico_sdk_init()

SET(ProjectId "BSP_test" CACHE STRING "Binary prefix name")
project(${ProjectId} C CXX ASM)
MESSAGE(STATUS "Configuring target: ${ProjectId}")

add_library(libbsp INTERFACE)

target_include_directories(libbsp INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/include)

add_subdirectory(drivers)
add_subdirectory(extras)

target_link_libraries(libbsp INTERFACE
	pico_stdlib
	hardware_adc
	hardware_uart 
    hardware_irq   
    hardware_flash
    hardware_rtc
    hardware_i2c
    hardware_spi
    hardware_timer

	bsp_drivers
	bsp_extras
)

add_executable(${ProjectId}
	bsp_test.c
)

target_compile_definitions(${ProjectId} PRIVATE PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
target_compile_options(${ProjectId} PRIVATE -Wall)

target_link_libraries(${ProjectId}
	libbsp
)

pico_enable_stdio_uart(${ProjectId} 0)
pico_enable_stdio_usb(${ProjectId}  1)
pico_add_extra_outputs(${ProjectId} )


