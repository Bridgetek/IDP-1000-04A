
add_library(i2c_utils INTERFACE)

target_include_directories(i2c_utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(i2c_utils INTERFACE hardware_i2c pico_stdlib)

target_compile_options(i2c_utils INTERFACE -Wall)

file(GLOB SRC_FILES
	"*.c"
)

# compile everything
target_sources(i2c_utils INTERFACE
	${SRC_FILES}
)