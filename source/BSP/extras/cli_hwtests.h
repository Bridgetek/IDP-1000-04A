/*
 * cli_hwtests.h - PDxx hardware test commands
 * Will include additional commands for FCC/FCT testing, if defined.
 *
 * 2022-12-06: adnan.jalaludin
 *
 */

#ifndef __CLI_HWTESTS_H__
#define __CLI_HWTESTS_H__

int cmd_i2cdetect(int argc, char *argv[]);
int cmd_rotsw(int argc, char *argv[]);
int cmd_ledtest(int argc, char *argv[]);
int cmd_alstest(int argc, char *argv[]);
int cmd_devices(int argc, char *argv[]);

#define CLI_HWTESTS_CMDLIST {"i2cdetect", cmd_i2cdetect}, \
							{"dev", cmd_devices},         \
							{"rot", cmd_rotsw},           \
							{"led", cmd_ledtest},         \
							{"als", cmd_alstest},


/* include additional FCC CLI testing commands */

#endif /* __CLI_HWTESTS_H__ */
