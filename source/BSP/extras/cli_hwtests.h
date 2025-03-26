/*
 * cli_hwtests.h - hardware test commands
 */

#ifndef __CLI_HWTESTS_H__
#define __CLI_HWTESTS_H__

int cmd_i2cdetect(int argc, char *argv[]);
int cmd_rotsw(int argc, char *argv[]);
int cmd_ledtest(int argc, char *argv[]);
int cmd_alstest(int argc, char *argv[]);
int cmd_devices(int argc, char *argv[]);
int cmd_sdcard(int argc, char* argv[]);
int cmd_evelogo(int argc, char* argv[]);
int cmd_buzzer(int argc, char* argv[]);

#define CLI_HWTESTS_CMDLIST {"i2cdetect", cmd_i2cdetect}, \
							{"dev", cmd_devices},         \
							{"rot", cmd_rotsw},           \
							{"led", cmd_ledtest},         \
							{"als", cmd_alstest},         \
                            {"sd", cmd_sdcard},           \
                            {"eve", cmd_evelogo},         \
                            {"buzzer", cmd_buzzer},

#endif /* __CLI_HWTESTS_H__ */
