#include <pico.h> // incs stdint,stdbool,stddefs,datetime_t
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <hardware/sync.h>
#include <hardware/watchdog.h>
#include <hardware/rtc.h>
#include <pico/bootrom.h>
#include <pico/stdio.h>
#include <pico/util/datetime.h> // only for datatime_to_str()

#include "cli.h"
#include "bsp_debug.h"
#include "bsp_hwdefs.h"

#include "bsp_app.h"
#include "bsp_util.h"
#include "cli_hwtests.h"

#include <libgen.h>

const char* month_strings[] = {
	"January", "February", "March", "April", "May", "June", "July",
	"August", "September", "October", "November", "December" };

static char	   kbuf[KBUF_MAX];
static uint8_t kbuf_count = 0;
static char	  *cliargs[CLI_MAX_TOKENS];
int			   errno;

/* Base commands */
int cmd_help(int argc, char *argv[]);
int cmd_clearscreen(int argc, char *argv[]);
int cmd_date(int argc, char *argv[]);
int cmd_picomem_dump(int argc, char *argv[]);
int cmd_reboot(int argc, char *argv[]);

// clang-format off
const CMD_ENTRY cmd_table[] = {
	{	"?",		&cmd_help			},
	{	"help",		&cmd_help			},
	{	"cls",		&cmd_clearscreen	},
	{	"date",		&cmd_date			},
	{	"mr",		&cmd_picomem_dump	},
	{	"reboot",	&cmd_reboot			},
		CLI_HWTESTS_CMDLIST
	{ 0, 0 },
};
// clang-format on

/* Quick and dirty getline
 *  - doesn't interpret escape codes
 *  - limits line entry to 72 chars to avoid backspace wraparound on last terminal line
 */
int nb_getline(void)
{
	int c = 0;
	int esc = 0;

	while ((c = getchar_timeout_us(0)) > 0) {

		switch (c) {
		case '\n':
		case '\r':
			putchar_raw('\n');
			kbuf[kbuf_count] = 0;
			return kbuf_count;
		case '\033':
			if (esc)
				esc = 0;
			else
				esc = 1;
			break;
		case '\b':
			if (esc) {
				esc = 0;
				// readline interpretes as kill-word
			}
			else {
				if (kbuf_count > 0) {
					putchar_raw(c);
					putchar_raw(' ');
					putchar_raw(c);
					kbuf_count--;
				}
			}
			break;
		default:
			if (esc == 1) {
				// interpret escape first char
				esc = 2;
			}
			else if (esc == 2) {
				// interpret escape second char
				esc = 0;
			}
			else if (isprint(c) && (kbuf_count < KBUF_MAX - 1)) {
				kbuf[kbuf_count++] = c;
				putchar_raw(c);
			}
			break;
		}
	}
	return -1;
}

/* cli_splitstring(char *p, char *argv[])
 * strtok() clone: except this one respects quoted arguments
 */
int cli_splitstring(char *p, char *argv[])
{
	int num_toks = 0;
	int state = ' ';
	while (*p) {
		switch (state) {
		case ' ':
			if (isspace((int)*p))
				break;
			argv[num_toks] = p;
			if (*p == '\'')
				state = 'Q'; // begin single-quote state
			else if (*p == '\"')
				state = 'D'; // begin double-quote state
			else
				state = 'T'; // begin normal alpha state
			break;
		case 'T':
			if (isspace((int)*p)) {
				// end of unquoted text
				*p = 0;
				num_toks++;
				state = ' '; // back to whitespace idle
			}
			break;
		case 'Q':
			if (*p == '\'') {
				// end of single-quoted text
				*p = 0;
				/* adjust prev-entered tok_start to remove leading quote */
				argv[num_toks] += 1;
				num_toks++;
				state = ' '; // back to whitespace idle
			}
			break;
		case 'D':
			if (*p == '\"') {
				// end of double-quoted text
				*p = 0;
				/* adjust prev-entered tok_start to remove leading quote */
				argv[num_toks] += 1;
				num_toks++;
				state = ' '; // back to whitespace idle
			}
			break;
		default:
			break;
		}
		p++;
	} // end while

	if (state != ' ') {
		num_toks++;
	}
	return num_toks;
}

static bool prompt_shown = false;

int do_nonblock_cli(void)
{
	int				 argc;
	const CMD_ENTRY *cmd_p;

	if (!prompt_shown) {
		memset(kbuf, 0, sizeof(kbuf));
		kbuf_count = 0;
		printf(CLI_PROMPT);
		prompt_shown = true;
	}

	/* if nb_getlne returns negative, no full line is collected yet */
	if (nb_getline() < 0)
		return 0;

	prompt_shown = false;
	argc = cli_splitstring(kbuf, cliargs);

	cmd_p = cmd_table;
	if (strnlen(cliargs[0], PATH_MAX) != 0) {
		while ((cmd_p->name != 0) && (strcmp(cmd_p->name, cliargs[0]) != 0)) {
			cmd_p++;
		}
		if (cmd_p->name != 0) {
			kbuf_count = 0;
			return (cmd_p->func(argc, cliargs));
		}
		printf("Command not found.\n");
	}

	return -1;
}

int anyopts(int argc, char *argv[], const char *needle)
{
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], needle) == 0)
			return i;
	}
	return -1;
}

int anynumopts(int argc, char *argv[], int start)
{
	for (int i = start; i < argc; i++) {
		if (*argv[i] == '-' && isdigit((int)*(argv[i] + 1))) {
			return i;
		}
	}
	return -1;
}

int lastopts(int argc, char *argv[])
{
	for (int i = argc - 1; (i > 0) && (i < argc); i--) {
		if (*argv[i] == '-') {
			return i;
		}
	}
	return 0;
}

int cmd_help(__unused int argc, __unused char *argv[])
{
	const CMD_ENTRY *cmd_p = cmd_table;
	printf("\nCommands available:\n");
	while (cmd_p->name != 0) {
		printf("%s, ", cmd_p->name);
		cmd_p++;
	}
	printf("\b\b. \n");
	return 0;
}

int cmd_date(int argc, char *argv[])
{
	char str[40];
	bool have_date = false;
	bool have_time = false;
	int	 year;
	int	 month;
	int	 day;
	int	 hour;
	int	 min;
	int	 sec;
	int	 toks;

	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			if (strchr(argv[i], '-')) {
				toks = sscanf(argv[i], "%d-%d-%d", &year, &month, &day);
				if (toks == 3 && year >= 2022 && month >= 1 && month <= 12 &&
					day >= 1 && day <= 31) {
					have_date = true;
				}
			}
			else if (strchr(argv[i], ':')) {
				toks = sscanf(argv[i], "%d:%d:%d", &hour, &min, &sec);
				if (toks == 3 && hour >= 0 && hour <= 23 && min >= 0 &&
					min <= 59 && sec >= 0 && sec <= 59) {
					have_time = true;
				}
				else if (toks == 2 && hour >= 0 && hour <= 23 && min >= 0 &&
						 min <= 59) {
					sec = 0;
					have_time = true;
				}
			}
		}
	}

	datetime_t dt = {0, 0, 0, 0, 0, 0, 0};
	rtc_get_datetime(&dt);
	if (have_date) {
		// https://stackoverflow.com/questions/6054016/c-program-to-find-day-of-week-given-date
		int y = year;
		int m = month;
		int d = day;
		int weekday = (d += m < 3 ? y-- : y - 2,
					   23 * m / 9 + d + 4 + y / 4 - y / 100 + y / 400) %
					  7;
		dt.year = (int16_t)year;
		dt.month = (int8_t)month;
		dt.day = (int8_t)day;
		dt.dotw = (int8_t)weekday;
	}
	if (have_time) {
		dt.hour = (int8_t)hour;
		dt.min = (int8_t)min;
		dt.sec = (int8_t)sec;
	}
	if (have_date || have_time) {
		rtc_set_datetime(&dt);
	}
	datetime_to_str(str, sizeof(str), &dt);
	printf("\n%s (%+02d:%02d)\n", str, dev_contexts_gptr->tz_hour,
		   dev_contexts_gptr->tz_min);
	return 0;
}

int cmd_clearscreen(__unused int argc, __unused char *argv[])
{
	printf("\033[H\033[J");
	return 0;
}

void pr_hex_dump(uint32_t addr, const uint8_t* buf, int len)
{
	int rounded_up_len = ALIGN_UP(16, len);
	int ascii_to_display = 16;

	for (int i = 0; i < rounded_up_len; i++) {
		if ((i % 16) == 0) {
			printf("%08lx ", addr + i);
		}
		if ((i % 8) == 0) {
			printf(" "); // print column spacer for 8-byte block
		}
		if (i < len) {
			printf("%02x ", buf[i]);
		}
		else {
			printf("   "); // exceeded, print spaceholders so sidebar looks aligned
		}
		if (((i + 1) % 16) == 0) {
			printf(" ");
			if (i >= len)
				ascii_to_display -= (rounded_up_len - len); // instead of 16 ascii, print less
			for (int j = 0; j < ascii_to_display; j++) {
				char c = buf[j + (i / 16) * 16];
				printf("%c", isprint(c) ? c : '.');
			}
			printf("\n");
		}
	}
}

int cmd_picomem_dump(int argc, char *argv[])
{
	const int block_len = 512;
	uint8_t	  buffer[block_len]; /* File copy buffer */
	bool	  havekey;
	bool	  is_flash = false;
	bool	  quit = false;
	uint32_t  addr = 0x0;
	int		  c;
	printf("\n");

	if ((anyopts(argc, argv, "-f") > 0) ||
		(anyopts(argc, argv, "-fl") > 0) ||
		(anyopts(argc, argv, "-flash") > 0)) {
		is_flash = true;
	}

	int addr_arg = lastopts(argc, argv) + 1;
	if (addr_arg < argc)
		addr = strtoul(argv[addr_arg], NULL, 0);
	// Do round off to 512-byte boundary
	addr &= ~(512 - 1);
	if (is_flash) {
		// as a convenience, to read Pico Flash minus the start address
		addr += 0x10000000; // Pico Flash start
	}

	do {
		uint32_t irqens = save_and_disable_interrupts();
		if (addr != 0) {
			memcpy(buffer, (uint8_t *)addr, block_len);
		}
		else {
			// it's been determined that location byte-addr 0 is 0
			*buffer = 0;
			memcpy(buffer + 1, (uint8_t *)1, block_len - 1);
		}
		restore_interrupts(irqens);

		pr_hex_dump(addr, buffer, block_len);

		printf("...");

		havekey = false;
		do {
			do {
				watchdog_update();
				c = getchar_timeout_us(0);
			} while (c < 0);
			switch (c) {
			case 'p':
				havekey = true;
				addr = is_flash ? 0x10000000 : 0;
				break;
			case '1':
				havekey = true;
				addr += 0x1000;
				break;
			case '2':
				havekey = true;
				addr += 0x10000;
				break;
			case '3':
				havekey = true;
				addr += 0x100000;
				break;
			case '!':
				havekey = true;
				addr -= 0x1000;
				break;
			case '@':
				havekey = true;
				addr -= 0x10000;
				break;
			case '#':
				havekey = true;
				addr -= 0x100000;
				break;
			case ' ':
			case 'n':
			case 'N':
				havekey = true;
				addr += block_len;
				break;
			case 'b':
			case 'B':
				havekey = true;
				addr -= block_len;
				break;
			case 'q':
			case 'Q':
			case 0x3:
				havekey = true;
				quit = true;
				break;
			default:
				break;
			}
		} while (!havekey);
		printf("\n");
	} while (!quit);

	return 0;
}

int cmd_reboot(int argc, char *argv[])
{
	if ((anyopts(argc, argv, "?") > 0) ||
		(anyopts(argc, argv, "-h") > 0)) {
		printf("Usage: %s [-u for USB upload mode]\n", argv[0]);
		return 0;
	}

	if (anyopts(argc, argv, "-u") > 0) {
		PR_INFO("%sMCU rebooting to USB-BootSel mode...%s\n\n", KGRN1, KNRM);
		reset_usb_boot(0, 0);
	}
	else {
		PR_INFO("%sMCU rebooting...%s\n\n", KGRN1, KNRM);
		sleep_ms(100);
		watchdog_enable(10, 1);
	}
	exit(0);
}
