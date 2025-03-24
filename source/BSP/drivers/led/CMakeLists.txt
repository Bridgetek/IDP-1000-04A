
add_library(kinetic_rgb_led INTERFACE)

target_include_directories(kinetic_rgb_led INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(kinetic_rgb_led INTERFACE hardware_i2c pico_stdlib)

target_compile_options(kinetic_rgb_led INTERFACE -Wall)

file(GLOB SRC_FILES
	"*.c"
)

# compile everything
target_sources(kinetic_rgb_led INTERFACE
	${SRC_FILES}
)