
# CMake configuration for multi-target EVE HAL build

add_library(eve_hal INTERFACE)

target_include_directories(eve_hal INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# Add the standard library to the build
TARGET_LINK_LIBRARIES(eve_hal INTERFACE pico_stdlib)

target_compile_options(eve_hal INTERFACE -Wall)

file(GLOB SRC_FILES
	"*.c"
)

# compile everything
target_sources(eve_hal INTERFACE
	${SRC_FILES}
)
