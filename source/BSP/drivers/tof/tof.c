#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/i2c.h>
#include "bsp_debug.h"
#include "bsp_hwdefs.h"
#include "tof.h"
#include "bsp_util.h"

bool tof_started = false;

uint8_t				  last_status;
static const uint32_t TimingGuard = 4528;
static const uint16_t TargetRate = 0x0A00;

typedef struct {
	uint8_t range_status;
	// uint8_t report_status: not used
	uint8_t	 stream_count;
	uint16_t dss_actual_effective_spads_sd0;
	// uint16_t peak_signal_count_rate_mcps_sd0: not used
	uint16_t ambient_count_rate_mcps_sd0;
	// uint16_t sigma_sd0: not used
	// uint16_t phase_sd0: not used
	uint16_t final_crosstalk_corrected_range_mm_sd0;
	uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
} ResultBuffer;

static ResultBuffer results;

uint32_t	 io_timeout;
bool		 did_timeout;
uint32_t	 timeout_start_ms;
uint16_t	 fast_osc_frequency;
uint16_t	 osc_calibrate_val;
bool		 calibrated;
uint8_t		 saved_vhv_init;
uint8_t		 saved_vhv_timeout;
DistanceMode distance_mode;
RangingData	 ranging_data;

static void updateDSS();
static void setupManualCalibration();
static void getRangingData();

uint32_t millis()
{
	return time_us_64() / 1000;
}

float countRateFixedToFloat(uint16_t count_rate_fixed)
{
	return (float)count_rate_fixed / (1 << 7);
}

static void startTimeout()
{
	timeout_start_ms = millis();
}

static bool checkTimeoutExpired()
{
	return (io_timeout > 0) && ((uint16_t)(millis() - timeout_start_ms) > io_timeout);
}

static int i2c_read(uint16_t cmd, uint8_t *buffer, uint8_t len)
{
	uint8_t cbuffer[2];
	cbuffer[0] = (cmd >> 8) & 0xff;
	cbuffer[1] = (cmd >> 0) & 0xff;
	int ret = i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, cbuffer, 2, true);
	ret = i2c_read_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, buffer, len, false);
	if (ret < 0) {
		// printf("TOF Read fail\n");
	}
	return ret;
}

static uint8_t readReg(uint16_t reg)
{
	uint8_t buffer[3];
	uint8_t value;
	buffer[0] = (reg >> 8);
	buffer[1] = (reg & 0xff);
	i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, buffer, 2, true);
	i2c_read_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, &value, 1, false);
	return value;
}

static int writeReg(uint16_t reg, uint8_t value)
{
	uint8_t buf[3];
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = value;
	int ret = i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, buf, 3, false);
	if (ret < 0) {
		printf("TOF WriteReg fail\n");
	}
	return ret;
}

static void writeReg16Bit(uint16_t reg, uint16_t value)
{
	uint8_t buf[4];
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = value >> 8;
	buf[3] = value & 0xff;
	int ret = i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, buf, 4, false);
	if (ret < 0) {
		printf("TOF WriteReg16 fail\n");
	}
}

static void writeReg32Bit(uint16_t reg, uint32_t value)
{
	uint8_t buf[6];
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = value >> 24;
	buf[3] = value >> 16;
	buf[4] = value >> 8;
	buf[5] = value & 0xff;
	int ret = i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, buf, 6, false);
	if (ret < 0) {
		printf("TOF WriteReg32 fail\n");
	}
}

static uint16_t readReg16Bit(uint16_t cmd)
{
	uint8_t value[2];
	uint8_t cbuffer[2];
	cbuffer[0] = (cmd >> 8) & 0xff;
	cbuffer[1] = (cmd >> 0) & 0xff;
	int ret = i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, cbuffer, 2, true);
	ret = i2c_read_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, value, 2, false);
	if (ret < 0) {
		printf("TOF ReadReg16 fail\n");
	}
	return ((value[0] << 8) | value[1]);
}

static uint32_t calcMacroPeriod(uint8_t vcsel_period)
{
	uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;
	uint8_t	 vcsel_period_pclks = (vcsel_period + 1) << 1;
	uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
	macro_period_us >>= 6;
	macro_period_us *= vcsel_period_pclks;
	macro_period_us >>= 6;
	return macro_period_us;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
	return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

static uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
	return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

static uint32_t decodeTimeout(uint16_t reg_val)
{
	return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

static uint16_t encodeTimeout(uint32_t timeout_mclks)
{
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}

		return (ms_byte << 8) | (ls_byte & 0xFF);
	}
	else {
		return 0;
	}
}

static uint32_t getMeasurementTimingBudget()
{
	uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));
	uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(
		decodeTimeout(readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);
	return 2 * range_config_timeout_us + TimingGuard;
}

static bool setMeasurementTimingBudget(uint32_t budget_us)
{
	if (budget_us <= TimingGuard) {
		return false;
	}
	uint32_t range_config_timeout_us = budget_us -= TimingGuard;
	if (range_config_timeout_us > 1100000) {
		return false;
	} // FDA_MAX_TIMING_BUDGET_US * 2
	range_config_timeout_us /= 2;
	uint32_t macro_period_us;
	macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));
	uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
	if (phasecal_timeout_mclks > 0xFF) {
		phasecal_timeout_mclks = 0xFF;
	}
	writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);
	writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A,
				  encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
	writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A,
				  encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us,
														   macro_period_us)));
	macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));
	writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B,
				  encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
	writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B,
				  encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us,
														   macro_period_us)));
	return true;
}

static bool setDistanceMode(DistanceMode mode)
{
	uint32_t budget_us = getMeasurementTimingBudget();
	switch (mode) {
	case Short:
		writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
		writeReg(SD_CONFIG__WOI_SD0, 0x07);
		writeReg(SD_CONFIG__WOI_SD1, 0x05);
		writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
		writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default
		break;

	case Medium:
		writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
		writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
		writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
		writeReg(SD_CONFIG__WOI_SD0, 0x0B);
		writeReg(SD_CONFIG__WOI_SD1, 0x09);
		writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
		writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default
		break;

	case Long: // long
		writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
		writeReg(SD_CONFIG__WOI_SD0, 0x0F);
		writeReg(SD_CONFIG__WOI_SD1, 0x0D);
		writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
		writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default
		break;

	default:
		return false;
	}
	setMeasurementTimingBudget(budget_us);
	distance_mode = mode;
	return true;
}

// read measurement results into buffer
static void readResults()
{
	uint8_t cbuffer[2];
	uint8_t status[32];
	cbuffer[0] = (RESULT__RANGE_STATUS >> 8) & 0xff;
	cbuffer[1] = (RESULT__RANGE_STATUS >> 0) & 0xff;
	i2c_write_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, cbuffer, 2, true);
	int ret = i2c_read_blocking(VL53L3CX_PICO_I2C_BUS, VL53L3CX_I2C_ADDR, status, 17, true);
	if (ret < 17) {
		printf("RANGE STATUS RAW DATA : READ FAIL\r\n");
		return;
	}
	// printf("RANGE STATUS RAW DATA : ");
	// print_buffer(status, 17);
	results.range_status = status[0];
	results.stream_count = status[2];
	results.dss_actual_effective_spads_sd0 = (uint16_t)status[3] << 8;						 // high byte
	results.dss_actual_effective_spads_sd0 |= status[4];									 // low byte
	results.ambient_count_rate_mcps_sd0 = (uint16_t)status[7] << 8;							 // high byte
	results.ambient_count_rate_mcps_sd0 |= status[8];										 // low byte
	results.final_crosstalk_corrected_range_mm_sd0 = (uint16_t)status[13] << 8;				 // high byte
	results.final_crosstalk_corrected_range_mm_sd0 |= status[14];							 // low byte
	results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = (uint16_t)status[15] << 8; // high byte
	results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |= status[16];				 // low byte
}

static int init(bool io_2v8)
{
	if (writeReg(SOFT_RESET, 0x00) < 0)
		return -1;
	sleep_ms(100);
	if (writeReg(SOFT_RESET, 0x01) < 0)
		return -1;
	sleep_ms(10);
	startTimeout();

	// check last_status in case we still get a NACK to try to deal with it correctly
	while ((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 || last_status != 0) {
		if (checkTimeoutExpired()) {
			did_timeout = true;
			return false;
		}
	}

	if (io_2v8) {
		writeReg(PAD_I2C_HV__EXTSUP_CONFIG,
				 readReg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
	}
	fast_osc_frequency = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
	osc_calibrate_val = readReg16Bit(RESULT__OSC_CALIBRATE_VAL);

	writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
	writeReg(GPIO__TIO_HV_STATUS, 0x02);
	writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);	   // tuning parm default
	writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
	writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	writeReg(ALGO__RANGE_MIN_CLIP, 0);				 // tuning parm default
	writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default
	writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
	writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
	writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);
	writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360);					 // tuning parm default
	writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default
	writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	writeReg(SD_CONFIG__QUANTIFIER, 2); // tuning parm default
	writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	writeReg(SYSTEM__SEED_CONFIG, 1);		 // tuning parm default
	writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
	writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS
	setDistanceMode(Long);
	setMeasurementTimingBudget(50000);
	writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
				  readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);
	return true;
}

bool dataReady() { return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0; }

static void startContinuous(uint32_t period_ms)
{
	writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);
	writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	writeReg(SYSTEM__MODE_START, 0x40);		 // mode_range__timed
}

static void stopContinuous()
{
	writeReg(SYSTEM__MODE_START, 0x80);
	calibrated = false;
	if (saved_vhv_init != 0) {
		writeReg(VHV_CONFIG__INIT, saved_vhv_init);
	}
	if (saved_vhv_timeout != 0) {
		writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
	}
	writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);
}

static uint16_t read(bool blocking)
{
	if (blocking) {
		startTimeout();
		while (!dataReady()) {
			if (checkTimeoutExpired()) {
				printf("Timeout...\r\n");
				did_timeout = true;
				return 0;
			}
		}
	}
	// printf("Read Results...\r\n");
	// readResults() is necessary to read data registers and process dist info
	readResults();

	if (!calibrated) {
		setupManualCalibration();
		calibrated = true;
	}

	updateDSS();
	// printf("Read Ranging data...\r\n");
	getRangingData();

	writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

	return ranging_data.range_mm;
}

static void setupManualCalibration()
{
	// "save original vhv configs"
	saved_vhv_init = readReg(VHV_CONFIG__INIT);
	saved_vhv_timeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

	// "disable VHV init"
	writeReg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

	// "set loop bound to tuning param"
	writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
			 (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

	// "override phasecal"
	writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
	writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
}

static void updateDSS()
{
	uint16_t spadCount = results.dss_actual_effective_spads_sd0;

	if (spadCount != 0) {
		// "Calc total rate per spad"

		uint32_t totalRatePerSpad =
			(uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
			results.ambient_count_rate_mcps_sd0;

		// "clip to 16 bits"
		if (totalRatePerSpad > 0xFFFF) {
			totalRatePerSpad = 0xFFFF;
		}

		// "shift up to take advantage of 32 bits"
		totalRatePerSpad <<= 16;

		totalRatePerSpad /= spadCount;

		if (totalRatePerSpad != 0) {
			// "get the target rate and shift up by 16"
			uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

			// "clip to 16 bit"
			if (requiredSpads > 0xFFFF) {
				requiredSpads = 0xFFFF;
			}

			// "override DSS config"
			writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
			// DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

			return;
		}
	}

	// If we reached this point, it means something above would have resulted in a
	// divide by zero.
	// "We want to gracefully set a spad target, not just exit with an error"

	// "set target to mid point"
	writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

// get range, status, rates from results buffer
// based on VL53L1_GetRangingMeasurementData()
static void getRangingData()
{
	// VL53L1_copy_sys_and_core_results_to_range_results() begin

	uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

	// "apply correction gain"
	// gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
	// Basically, this appears to scale the result by 2011/2048, or about 98%
	// (with the 1024 added for proper rounding).
	ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

	// VL53L1_copy_sys_and_core_results_to_range_results() end

	// set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
	// mostly based on ConvertStatusLite()
	switch (results.range_status) {
	case 17: // MULTCLIPFAIL
	case 2:	 // VCSELWATCHDOGTESTFAILURE
	case 1:	 // VCSELCONTINUITYTESTFAILURE
	case 3:	 // NOVHVVALUEFOUND
		// from SetSimpleData()
		ranging_data.range_status = HardwareFail;
		break;

	case 13: // USERROICLIP
			 // from SetSimpleData()
		ranging_data.range_status = MinRangeFail;
		break;

	case 18: // GPHSTREAMCOUNT0READY
		ranging_data.range_status = SynchronizationInt;
		break;

	case 5: // RANGEPHASECHECK
		ranging_data.range_status = OutOfBoundsFail;
		break;

	case 4: // MSRCNOTARGET
		ranging_data.range_status = SignalFail;
		break;

	case 6: // SIGMATHRESHOLDCHECK
		ranging_data.range_status = SigmaFail;
		break;

	case 7: // PHASECONSISTENCY
		ranging_data.range_status = WrapTargetFail;
		break;

	case 12: // RANGEIGNORETHRESHOLD
		ranging_data.range_status = XtalkSignalFail;
		break;

	case 8: // MINCLIP
		ranging_data.range_status = RangeValidMinRangeClipped;
		break;

	case 9: // RANGECOMPLETE
		// from VL53L1_copy_sys_and_core_results_to_range_results()
		if (results.stream_count == 0) {
			ranging_data.range_status = RangeValidNoWrapCheckFail;
		}
		else {
			ranging_data.range_status = RangeValid;
		}
		break;

	default:
		ranging_data.range_status = None;
	}

	// from SetSimpleData()
	ranging_data.peak_signal_count_rate_MCPS =
		countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
	ranging_data.ambient_count_rate_MCPS =
		countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()

static bool dev_initialised = false;

void tof_pins_init()
{
	gpio_init(VL53L3CX_RST_PICO_PIN);
	gpio_set_dir(VL53L3CX_RST_PICO_PIN, GPIO_OUT);
	gpio_init(VL53L3CX_IRQ_PICO_PIN);
	gpio_set_dir(VL53L3CX_IRQ_PICO_PIN, GPIO_IN);

	gpio_put(VL53L3CX_RST_PICO_PIN, 0);
	sleep_ms(100);
	gpio_put(VL53L3CX_RST_PICO_PIN, 1);
	sleep_ms(100);
}

int tof_init()
{
	uint8_t model = 0x07;
	uint8_t module_type = 0;
	int		ret = i2c_read(IDENTIFICATION__MODEL_ID, &model, 1);
	ret = i2c_read(IDENTIFICATION__MODULE_TYPE, &module_type, 1);
	if (ret < 0) {
		// PR_ERROR("%s(): VL53L3CX TOF not found\n", __func__);
		return -1;
	}

	const bool io_is_2V8 = false;
	if (init(io_is_2V8) < 0) {
		return -1;
	}
	dev_initialised = true;
	setDistanceMode(Long);
	setMeasurementTimingBudget(50000);
	startContinuous(50);
	tof_started = true;
	return 0;
}

void tof_stop(void)
{
	if (tof_started) {
		stopContinuous();
		tof_started = false;
	}
}

uint16_t tof_get_mm(void)
{
	if (!dev_initialised) {
		PR_ERROR("%s(): TOF is unintialized\n", __func__);
		return 0;
	}
	// printf("%s(): irqpin=%d\n", __func__, gpio_get(VL53L3CX_IRQ_PICO_PIN));
	return read(false);
}
