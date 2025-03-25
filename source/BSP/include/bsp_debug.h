#ifndef _BSP_DEBUG_H_
#define _BSP_DEBUG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define KNRM	"\e[0m"
#define KRED	"\e[91m"
#define KGRN	"\e[32m"
#define KGRN1	"\e[1;32m"
#define KYEL	"\e[33m"
#define KBLU	"\e[34m"
#define KBLUDIR "\e[94m"

#define PR_ERROR(...)                             \
	do {                                          \
		printf("%s%s", KGRN, KRED); \
		printf(__VA_ARGS__);                      \
		printf("%s", KNRM);                       \
	} while (0)

#define PR_WARN(...)                              \
	do {                                          \
		printf("%s%s", KGRN, KYEL); \
		printf(__VA_ARGS__);                      \
		printf("%s", KNRM);                       \
	} while (0)

#define PR_INFO(...)                              \
	do {                                          \
		printf("%s%s", KGRN, KGRN); \
		printf(__VA_ARGS__);                      \
		printf("%s", KNRM);                       \
	} while (0)

/* To enable/disable this flag
 * cmake -DDBG_PRINT=true/false -B build -S <src path>
 */
#ifdef DBG_PRINT
#define DPRINTF_ERROR(fmt, ...)           \
	do {                                  \
		printf("%s", KRED); \
		printf(fmt, ##__VA_ARGS__);       \
		printf("%s", KNRM);               \
	} while (0);

#define DPRINTF_WARN(fmt, ...)            \
	do {                                  \
		printf("%s", KYEL); \
		printf(fmt, ##__VA_ARGS__);       \
		printf("%s", KNRM);               \
	} while (0);

#define DPRINTF_INFO(fmt, ...)            \
	do {                                  \
		printf("%s", KGRN); \
		printf(fmt, ##__VA_ARGS__);       \
		printf("%s", KNRM);               \
	} while (0);
// This is usefull in while(1) loop
#define DPRINT_DGB_ONETIME(fmt, ...)          \
	do {                                      \
		static int dbg_count = 0;             \
		if (dbg_count == 0) {                 \
			printf("%s", KGRN); \
			printf(fmt, ##__VA_ARGS__);       \
			printf("%s", KNRM);               \
			dbg_count++;                      \
		}                                     \
	} while (0);
#else
#define DPRINTF_ERROR(fmt, ...)
#define DPRINTF_WARN(fmt, ...)
#define DPRINTF_INFO(fmt, ...)
#define DPRINT_DGB_ONETIME(fmt, ...)
#endif /* DBG_PRINT */

#endif /* _BSP_DEBUG_H_ */
