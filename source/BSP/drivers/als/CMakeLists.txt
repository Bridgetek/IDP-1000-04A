
add_library(als INTERFACE)

target_include_directories(als INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(als INTERFACE hardware_i2c pico_stdlib)

target_compile_options(als INTERFACE -Wall)

file(GLOB SRC_FILES
	"*.c"
)

# compile everything
target_sources(als INTERFACE
	${SRC_FILES}
)