/*
 * cli.h
 *
 *  Created on: 4 May 2021
 *      Author: adnan.jalaludin
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


extern int nb_getline(void);
extern int do_nonblock_cli(void);
extern int cli_splitstring(char *p, char *argv[]);
extern int anyopts(int argc, char *argv[], const char *needle);
extern int anynumopts(int argc, char *argv[], int start);
extern int lastopts(int argc, char *argv[]);

extern void pr_hex_dump(uint32_t addr, const uint8_t *buf, int len);

#endif /* __CLI_H__ */
