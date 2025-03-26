/*
 * cli.h
 */

#ifndef __CLI_H__
#define __CLI_H__

#include <stdint.h>
#include <stdbool.h>

#define KBUF_MAX	   1024
#define CLI_MAX_TOKENS 32
#define FPNAME_MAXLEN  255
#ifndef PATH_MAX
#define PATH_MAX FPNAME_MAXLEN
#endif

#define CLI_PROMPT "\e[1;32mshell> \e[0m"

typedef struct CMD_ENTRY {
	const char *name;
	const int (*func)(int argc, char *argv[]);
} CMD_ENTRY;


int do_nonblock_cli(void);
int anyopts(int argc, char *argv[], const char *needle);

#endif /* __CLI_H__ */
