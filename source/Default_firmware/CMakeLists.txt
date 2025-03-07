# CMake project ################################################################
# Build instruction:
#  set PICO_SDK_PATH=[path-to-pico-sdk]
#  set PICO_TOOLCHAIN_PATH=[path-to-gnu-arm-toolchain]
#  cd build && cmake  -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
#  nmake App
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.13 FATAL_ERROR)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Ensure that we are not building in our source directories. ###################
SET(Build_Dir_OK "TRUE")
STRING(REGEX MATCH "^${CMAKE_SOURCE_DIR}" In_Sub_Dir ${CMAKE_BINARY_DIR})
IF (In_Sub_Dir)
  STRING(REGEX MATCH "^${CMAKE_SOURCE_DIR}/[Bb]uild" In_Build_Dir ${CMAKE_BINARY_DIR})
  IF (NOT In_Build_Dir)
    SET(Build_Dir_OK "FALSE")
  ENDIF ()
ENDIF ()

IF (NOT Build_Dir_OK)
  MESSAGE(FATAL_ERROR "You must run cmake from a directory that is not in your source tree, or that is in a special subdirectory of the tree whose name begins with 'build'.")
ENDIF ()
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/") 

# Platform SDk setting #########################################################
function(validate_env_rp2040 env env_name)
    MESSAGE(STATUS "${env_name} is ${env}")
    
    if ("${env}" STREQUAL "")
        MESSAGE(FATAL_ERROR "${env_name} is not set, stop compile!")
    endif()
endfunction()

validate_env_rp2040("$ENV{PICO_SDK_PATH}" "PICO_SDK_PATH")
validate_env_rp2040("$ENV{PICO_TOOLCHAIN_PATH}" "PICO_TOOLCHAIN_PATH")

# Pull in Pico SDK (must be before project)
INCLUDE(pico_sdk_import.cmake)

include(${PICO_SDK_INIT_CMAKE_FILE})

# Initialise the Pico SDK
pico_sdk_init()

SET(ProjectId "IDP2040_101A" CACHE STRING "Binary prefix name")
project(${ProjectId} C CXX ASM)
MESSAGE(STATUS "Configuring target: ${ProjectId}")

# Project setup ################################################################
include_directories(lib/fatfs/source
eve_hal)
LINK_LIBRARIES(
  pico_stdlib 
  hardware_uart 
  hardware_irq   
  hardware_flash
  hardware_rtc
  hardware_i2c
  hardware_spi
  hardware_timer
  )

# Iterate through all sub folder and add into ADD_SUBDIRECTORY
FILE(GLOB SRCS 
  eve_hal/*.c 
  lib/*.c 
  lib/fatfs/source/*.c
  *.c)
FILE(GLOB HDRS 
  eve_hal/*.h 
  lib/*.h 
  lib/fatfs/source/*.h
  *.h)

ADD_EXECUTABLE(${ProjectId} ${SRCS} ${HDRS})

pico_enable_stdio_uart(${ProjectId} 0)
pico_enable_stdio_usb(${ProjectId}  1)
pico_add_extra_outputs(${ProjectId} )

