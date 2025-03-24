
add_library(efm8_mio INTERFACE)

target_include_directories(efm8_mio INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(efm8_mio INTERFACE hardware_i2c pico_stdlib)

target_compile_options(efm8_mio INTERFACE -Wall)

file(GLOB SRC_FILES
	"*.c"
)

# compile everything
target_sources(efm8_mio INTERFACE
	${SRC_FILES}
)