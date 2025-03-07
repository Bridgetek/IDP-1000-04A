
FILE(GLOB SRCS *.c)
FILE(GLOB HDRS *.h)
SOURCE_GROUP("" FILES ${SRCS} ${HDRS})

MESSAGE(STATUS "Configuring library: fatfs")

ADD_LIBRARY(fatfs
    ${SRCS}
    ${HDRS}
  )

TARGET_LINK_LIBRARIES(
  fatfs
  pico_stdlib
  hardware_gpio
  hardware_spi
  hardware_timer
  hardware_clocks
)
