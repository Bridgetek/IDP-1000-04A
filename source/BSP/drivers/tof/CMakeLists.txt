
add_library(tof INTERFACE)

target_include_directories(tof INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(tof INTERFACE hardware_i2c pico_stdlib)

target_compile_options(tof INTERFACE -Wall)

file(GLOB SRC_FILES
	"*.c"
)

# compile everything
target_sources(tof INTERFACE
	${SRC_FILES}
)