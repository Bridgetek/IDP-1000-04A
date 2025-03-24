#include <pico.h> // incs stdint,stdbool,stddefs,datetime_t
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pico/stdio.h>
#include <hardware/watchdog.h>

#include "cli.h"
#include "bsp_app.h"
#include "bsp_util.h"
#include "cli_hwtests.h"

#include "i2c_utils.h"

#include "efm8_mio.h"
#include "als.h"
#include "tof.h"
#include "rgbled.h"

static bool reserved_addr(uint8_t addr)
{
	return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int cmd_i2cdetect(int argc, char *argv[])
{
	const char *cmdp = argv[0];
	int			busnum = 0;

	if (anyopts(argc, argv, "-h") > 0 || argc > 2) {
		goto USAGE;
	}
	if (argc > 1) {
		busnum = strtoul(argv[1], NULL, 0);
		busnum = BOUND(0, busnum, 1);
	}

	// https://www.digikey.sg/en/maker/projects/raspberry-pi-pico-rp2040-i2c
	// -example-with-micropython-and-cc/47d0c922b79342779cdbd4b37b7eb7e2
	printf("\nI2C Bus Scan of bus %d\n", busnum);
	printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (int addr = 0; addr < (1 << 7); ++addr) {
		if (addr % 16 == 0) {
			printf("%02x ", addr);
		}
		// Perform a 1-byte dummy read from the probe address. If a slave
		// acknowledges this address, the function returns the number of bytes
		// transferred. If the address byte is ignored, the function returns
		// -1.
		// Skip over any reserved addresses.
		int		ret;
		uint8_t rxdata;
		if (reserved_addr(addr))
			ret = PICO_ERROR_GENERIC;
		else
			ret = i2c_read_blocking(busnum ? i2c1 : i2c0, addr, &rxdata, 1, false);
		printf(ret < 0 ? "." : "@");
		printf(addr % 16 == 15 ? "\n" : "  ");
	}
	printf("Done.\n");
	return 0;
USAGE:
	printf("%s: program to scan an I2C bus for devices.\n"
		   "    Options:\n"
		   "\t-h Prints this help\n"
		   "    %s [options] [0|1] Scan bus number (default=0)\n",
		   cmdp, cmdp);
	return 0;
}

int cmd_devices(int argc, char *argv[])
{
	BSPContexts_t*dev = dev_contexts_gptr;
	printf("\nRotary switch ID = %d\n", efm8_get_rotary());
	printf("MCU temperature = %g degC\n", dev->dev_temperature);
	printf("TOF distance = %d mm\n", dev->dev_tof_distance);
	printf("ALS brightness = %d lux\n", dev->dev_als_lux);
	printf("    auto brightness = %d%%\n", dev->dev_auto_pwm);

	return 0;
}


int cmd_rotsw(int argc, char *argv[])
{
	int res = efm8_get_rotary();

	printf("\n%s: efm8_get_rotary() res=%d\n", argv[0], res);

	return 0;
}

int cmd_ledtest(int argc, char *argv[])
{
	const char *cmdp = argv[0];
	uint32_t	color;

	if (anyopts(argc, argv, "-h") > 0 || argc < 2) {
		goto USAGE;
	}
	color = strtoul(argv[1], NULL, 16);

	printf("\nSetting LED to 0x%06lX\n", color & 0xffffff);
	led_strip_selection_both();
	led_set(color, 100);
	return 0;
USAGE:
	printf("\n%s [options] <color>  modify Panel LED parameters\n"
		   "\tcolor is RGB in hex\n"
		   "    Options:\n"
		   "      -h Prints this help\n",
		   cmdp);
	return 0;
}

int cmd_alstest(int argc, char *argv[])
{
	int		 c;
	bool	 exit = 0;
	uint16_t lux_reading, tof_reading;

	tof_pins_init();
	// printf("TOF resetted.\n");
	i2c_pins_init();
	// printf("EFM8 initialised.\n");
	int alsdvr_ok = als_init();
	// printf("ALS initialised.\n");
	int tofdvr_ok = tof_init();

	printf("\n%s: alsdvr_ok=%d, tofdvr_ok=%d\n", argv[0], alsdvr_ok, tofdvr_ok);
	printf("Press ctrl-C or esc to stop\n");

	do {
		watchdog_update();

		int als_read_ok = als_getlux(&lux_reading);
		tof_reading = tof_get_mm();
		printf("als_read_ok=%d: lux=%d tof=%d mm\n", als_read_ok, lux_reading, tof_reading);

		sleep_ms(100);
		c = getchar_timeout_us(0);

		if ((c == 'q') || (c == 'Q') || (c == 0x03) || (c == 27)) {
			exit = true;
		}
	} while (!exit);

	return 0;
}