
# CMake configuration for multi-target EVE HAL build

FILE(GLOB SRCS *.c)
FILE(GLOB HDRS *.h)
SOURCE_GROUP("" FILES ${SRCS} ${HDRS})

FILE(GLOB HDRS_EXTERNAL Hdr/*.h)
SOURCE_GROUP("Hdr" FILES ${HDRS_EXTERNAL})

INCLUDE_DIRECTORIES(./)
INCLUDE_DIRECTORIES(Hdr)

ADD_LIBRARY(eve_hal
  ${SRCS}
  ${HDRS}
  ${HDRS_EXTERNAL}
)

# Add the standard library to the build
TARGET_LINK_LIBRARIES(eve_hal pico_stdlib)
TARGET_LINK_LIBRARIES(eve_hal fatfs)
