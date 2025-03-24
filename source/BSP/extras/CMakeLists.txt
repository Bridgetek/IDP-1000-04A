add_library(bsp_extras INTERFACE)

target_include_directories(bsp_extras INTERFACE
	${CMAKE_CURRENT_LIST_DIR})

target_compile_options(bsp_extras INTERFACE -Wall)

# Compile everything in this directory
file(GLOB SRC_FILES
	"*.c"
)
target_sources(bsp_extras INTERFACE

	# ${SRC_FILES}
	cli_basic.c
	cli_hwtests.c
)
