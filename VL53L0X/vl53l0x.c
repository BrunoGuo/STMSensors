/**
 * @file vl53l0x.c
 *
 * @author Daniel Nery <daniel.nery@thunderatz.org>
 *
 * Equipe ThundeRatz de Robotica
 *
 * @date 03/2017
 *
 * @see https://github.com/pololu/vl53l0x-arduino
 */

#include "vl53l0x.h"
#include "string.h"

/*****************************************
 * Private Constants
 *****************************************/

#define VL53L0X_I2C_TIMEOUT 1000

/*****************************************
 * Private Macros
 *****************************************/

#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks)*1655) + 500) / 1000)

#define startTimeout() (timeout_start_ms = HAL_GetTick())
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)HAL_GetTick() - timeout_start_ms) > io_timeout)

/*****************************************
 * Private Types
 *****************************************/

typedef struct {
    uint8_t tcc;
    uint8_t msrc;
    uint8_t dss;
    uint8_t pre_range;
    uint8_t final_range;
} SequenceStepEnables;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks;
    uint16_t final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint16_t pre_range_mclks;
    uint16_t final_range_mclks;
    uint32_t msrc_dss_tcc_us;
    uint32_t pre_range_us;
    uint32_t final_range_us;
} SequenceStepTimeouts;

typedef enum {
    SYSRANGE_START = 0x00,

    SYSTEM_THRESH_HIGH = 0x0C,
    SYSTEM_THRESH_LOW = 0x0E,

    SYSTEM_SEQUENCE_CONFIG = 0x01,
    SYSTEM_RANGE_CONFIG = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

    GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

    SYSTEM_INTERRUPT_CLEAR = 0x0B,

    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS = 0x14,

    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

    ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

    I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

    MSRC_CONFIG_CONTROL = 0x60,

    PRE_RANGE_CONFIG_MIN_SNR = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

    FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

    PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

    PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

    SYSTEM_HISTOGRAM_BIN = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

    FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

    MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

    SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
    IDENTIFICATION_MODEL_ID = 0xC0,
    IDENTIFICATION_REVISION_ID = 0xC2,

    OSC_CALIBRATE_VAL = 0xF8,

    GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

    GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

    ALGO_PHASECAL_LIM = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
} vl53l0x_regAddr;

typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;

/*****************************************
 * Private Variables
 *****************************************/

static uint8_t address = VL53L0X_DEFAULT_ADDRESS; /**< Current Address */
static uint8_t did_timeout = 0;                   /**< Timeout detection */
static uint16_t io_timeout = 500;                 /**< Timeout in miliseconds */

static uint8_t stop_variable;
static uint8_t timeout_start_ms;
static uint32_t measurement_timing_budget_us;

static HAL_StatusTypeDef status;

/*****************************************
 * Private Function Prototypes
 *****************************************/

/**
 * @brief HAL I2C wrapper functions.
 */
static void writeReg(uint8_t reg, uint8_t val);
static void writeReg16(uint8_t reg, uint16_t val);
static void writeReg32(uint8_t reg, uint32_t val);
static void writeMulti(uint8_t reg, uint8_t* src, uint8_t count);
static uint8_t readReg(uint8_t reg);
static uint16_t readReg16(uint8_t reg);
static uint32_t readReg32(uint8_t reg);
static void readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

/**
 * @brief Get reference SPAD (single photon avalanche diode) count and type.
 *
 * @note based on VL53L0X_get_info_from_device(), but only gets reference SPAD count and type.
 *
 * @param count Pointer where SPAD count will be stored.
 * @param type_is_aperture Pointer where SPAD type will be stored.
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t getSpadInfo(uint8_t* count, uint8_t* type_is_aperture);

/**
 * @brief Set the measurement timing budget
 *
 * @note Time allowed for one measurement; the ST API and this library take
 *       care of splitting the timing budget among the sub-steps in the ranging sequence.
 *       A longer timing budget allows for more accurate measurements. Increasing the
 *       budget by a factor of N decreases the range measurement standard deviation
 *       by a factor of sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
 *       based on VL53L0X_set_measurement_timing_budget_micro_seconds()
 *
 * @param budget_us Timing budget in microseconds.
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t setMeasurementTimingBudget(uint32_t budget_us);

/**
 * @brief Get the measurement timing budget
 *
 * @note based on VL53L0X_get_measurement_timing_budget_micro_seconds()
 *
 * @return Timing budget in microseconds.
 */
uint32_t getMeasurementTimingBudget();

/**
 * @brief Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
 *        given period type (pre-range or final range) to the given value in PCLKs.
 *
 * @note Longer periods seem to increase the potential range of the sensor.
 *       Valid values are (even numbers only):
 *        pre:  12 to 18 (initialized default: 14)
 *        final: 8 to 14 (initialized default: 10)
 *       based on VL53L0X_set_vcsel_pulse_period()
 *
 * @param type Period Type (VcselPeriodPreRange or VcselPeriodFinalRange).
 * @param period_pclks Period in PCLKs
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

/**
 * @brief Get the VCSEL pulse period
 *
 * @note based on VL53L0X_get_vcsel_pulse_period()
 *
 * @param type Period Type (VcselPeriodPreRange or VcselPeriodFinalRange).
 *
 * @return Period in PCLKs.
 */
uint8_t getVcselPulsePeriod(vcselPeriodType type);

/**
 * @brief Get sequence step enables
 *
 * @note based on VL53L0X_GetSequenceStepEnables()
 *
 * @param enables Pointer to SequenceStepEnables struct where
 *        values will be stored.
 */
void getSequenceStepEnables(SequenceStepEnables* enables);

/**
 * @brief Get sequence step timeouts
 *
 * @note based on get_sequence_step_timeout(), but gets all timeouts
 *       instead of just the requested one, and also stores
 *       intermediate values
 *
 * @param enables Pointer to initialized SequenceStepEnables struct;
 * @param timeout Pointer to SequenceStepTimeouts struct where values
 *        will be stored.
 */
void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts);

/**
 * @brief Decode sequence step timeout in MCLKs from register value.
 *
 * @note Based on VL53L0X_decode_timeout()
 * @note The original function returned a uint32_t, but the return value is
 *       always stored in a uint16_t.
 *
 * @param reg_val Register value.
 *
 * @return Decoded value in MCLKs.
 */
uint16_t decodeTimeout(uint16_t reg_val);

/**
 * @brief Encode sequence step timeout register value from timeout in MCLKs.
 *
 * @note Based on VL53L0X_encode_timeout()
 * @note the original function took a uint32_t, but the argument passed to it
 *       is always a uint16_t.
 *
 * @param timeout_mclks Timeout in MCLKs.
 *
 * @return Encoded value.
 */
uint16_t encodeTimeout(uint16_t timeout_mclks);

/**
 * @brief Convert sequence step timeout from MCLKs to microseconds with given VCSEL
 *        period in PCLKs
 *
 * @note Based on VL53L0X_calc_timeout_us().

 * @param timeout_period_mclks Timeout in MCLKs.
 * @param vcsel_period_pclks VCSEL period in PCLKs.
 *
 * @return Timeout in microseconds.
 */
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

/**
 * @brief Convert sequence step timeout from microseconds to MCLKs with given VCSEL
 *        period in PCLKs
 *
 * @note Based on VL53L0X_calc_timeout_mclks().

 * @param timeout_period_us Timeout in microseconds.
 * @param vcsel_period_pclks VCSEL period in PCLKs.
 *
 * @return Timeout in MCLKs.
 */
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

/**
 * @brief Performs Calibration
 *
 * @note Based on VL53L0X_perform_single_ref_calibration().

 * @param vhv_init_byte
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t performSingleRefCalibration(uint8_t vhv_init_byte);

/*****************************************
 * Public Function Body Definitions
 *****************************************/

void vl53l0x_setCurrentAddress(uint8_t new_addr) {
    address = new_addr;
}

void vl53l0x_setDevAddress(uint8_t new_addr) {
    writeReg(I2C_SLAVE_DEVICE_ADDRESS, (new_addr >> 1) & 0x7F);
}

uint8_t vl53l0x_init() {
    writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);

    writeReg(0x88, 0x00);

    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    stop_variable = readReg(0x91);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit
    // checks
    writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

    writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0.25 * (1 << 7));

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // VL53L0X_StaticInit() begin

    uint8_t spad_count;
    uint8_t spad_type_is_aperture;
    if (!getSpadInfo(&spad_count, &spad_type_is_aperture))
        return 0;

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device()
    // in the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    writeReg(0xFF, 0x01);
    writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    writeReg(0xFF, 0x00);
    writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;  // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            spads_enabled++;
    }

    writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);

    writeReg(0xFF, 0x00);
    writeReg(0x09, 0x00);
    writeReg(0x10, 0x00);
    writeReg(0x11, 0x00);

    writeReg(0x24, 0x01);
    writeReg(0x25, 0xFF);
    writeReg(0x75, 0x00);

    writeReg(0xFF, 0x01);
    writeReg(0x4E, 0x2C);
    writeReg(0x48, 0x00);
    writeReg(0x30, 0x20);

    writeReg(0xFF, 0x00);
    writeReg(0x30, 0x09);
    writeReg(0x54, 0x00);
    writeReg(0x31, 0x04);
    writeReg(0x32, 0x03);
    writeReg(0x40, 0x83);
    writeReg(0x46, 0x25);
    writeReg(0x60, 0x00);
    writeReg(0x27, 0x00);
    writeReg(0x50, 0x06);
    writeReg(0x51, 0x00);
    writeReg(0x52, 0x96);
    writeReg(0x56, 0x08);
    writeReg(0x57, 0x30);
    writeReg(0x61, 0x00);
    writeReg(0x62, 0x00);
    writeReg(0x64, 0x00);
    writeReg(0x65, 0x00);
    writeReg(0x66, 0xA0);

    writeReg(0xFF, 0x01);
    writeReg(0x22, 0x32);
    writeReg(0x47, 0x14);
    writeReg(0x49, 0xFF);
    writeReg(0x4A, 0x00);

    writeReg(0xFF, 0x00);
    writeReg(0x7A, 0x0A);
    writeReg(0x7B, 0x00);
    writeReg(0x78, 0x21);

    writeReg(0xFF, 0x01);
    writeReg(0x23, 0x34);
    writeReg(0x42, 0x00);
    writeReg(0x44, 0xFF);
    writeReg(0x45, 0x26);
    writeReg(0x46, 0x05);
    writeReg(0x40, 0x40);
    writeReg(0x0E, 0x06);
    writeReg(0x20, 0x1A);
    writeReg(0x43, 0x40);

    writeReg(0xFF, 0x00);
    writeReg(0x34, 0x03);
    writeReg(0x35, 0x44);

    writeReg(0xFF, 0x01);
    writeReg(0x31, 0x04);
    writeReg(0x4B, 0x09);
    writeReg(0x4C, 0x05);
    writeReg(0x4D, 0x04);

    writeReg(0xFF, 0x00);
    writeReg(0x44, 0x00);
    writeReg(0x45, 0x20);
    writeReg(0x47, 0x08);
    writeReg(0x48, 0x28);
    writeReg(0x67, 0x00);
    writeReg(0x70, 0x04);
    writeReg(0x71, 0x01);
    writeReg(0x72, 0xFE);
    writeReg(0x76, 0x00);
    writeReg(0x77, 0x00);

    writeReg(0xFF, 0x01);
    writeReg(0x0D, 0x01);

    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x01);
    writeReg(0x01, 0xF8);

    writeReg(0xFF, 0x01);
    writeReg(0x8E, 0x01);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg(GPIO_HV_MUX_ACTIVE_HIGH,
             readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);  // active low
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end4

    measurement_timing_budget_us = getMeasurementTimingBudget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    setMeasurementTimingBudget(measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(0x40))
        return 0;

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(0x00))
        return 0;

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    setVcselPulsePeriod(VcselPeriodPreRange, VL53L0X_VCSEL_PRE_RANGE);
    setVcselPulsePeriod(VcselPeriodFinalRange, VL53L0X_VCSEL_FINAL_RANGE);

    return 1;
}

void vl53l0x_startContinuous(uint32_t period_ms) {
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stop_variable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    if (period_ms != 0) {
        // continuous timed mode
        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
        uint16_t osc_calibrate_val = readReg16(OSC_CALIBRATE_VAL);
        if (osc_calibrate_val != 0)
            period_ms *= osc_calibrate_val;

        writeReg32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

        writeReg(SYSRANGE_START, 0x04);  // VL53L0X_REG_SYSRANGE_MODE_TIMED
    } else {
        // continuous back-to-back mode
        writeReg(SYSRANGE_START, 0x02);  // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void vl53l0x_stopContinuous() {
    writeReg(SYSRANGE_START, 0x01);  // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, 0x00);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
uint16_t vl53l0x_getRange() {
    startTimeout();
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired()) {
            did_timeout = 1;
            return VL53L0X_TIMEOUT_RETURN_VALUE;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    uint16_t range = readReg16(RESULT_RANGE_STATUS + 10);

    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t vl53l0x_getRangeSingle() {
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, stop_variable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    writeReg(SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    startTimeout();
    while (readReg(SYSRANGE_START) & 0x01) {
        if (checkTimeoutExpired()) {
            did_timeout = 1;
            return VL53L0X_TIMEOUT_RETURN_VALUE;
        }
    }

    return vl53l0x_getRange();
}

/*****************************************
 * Public Function Body Definitions
 *****************************************/

void writeReg(uint8_t reg, uint8_t val) {
    uint8_t bytes[2] = {reg, val};

    while (HAL_I2C_Master_Transmit(&VL53L0X_I2C_HANDLER, address, (uint8_t*)(&bytes), 2, VL53L0X_I2C_TIMEOUT) != HAL_OK)
        ;
}

void writeReg16(uint8_t reg, uint16_t val) {
    uint8_t bytes[3] = {reg, (val >> 8) & 0xFF, val & 0xFF};

    while (HAL_I2C_Master_Transmit(&VL53L0X_I2C_HANDLER, address, (uint8_t*)(&bytes), 3, VL53L0X_I2C_TIMEOUT) != HAL_OK)
        ;
}

void writeReg32(uint8_t reg, uint32_t val) {
    uint8_t bytes[5] = {reg, (val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF};

    while (HAL_I2C_Master_Transmit(&VL53L0X_I2C_HANDLER, address, (uint8_t*)(&bytes), 5, VL53L0X_I2C_TIMEOUT) != HAL_OK)
        ;
}

void writeMulti(uint8_t reg, uint8_t* src, uint8_t count) {
    if (count > 31)
        return;

    count++;
    uint8_t bytes[32] = {0};
    bytes[0] = reg;
    for (uint8_t i = 1; i < count; i++)
        bytes[i] = src[i - 1];

    while (HAL_I2C_Master_Transmit(&VL53L0X_I2C_HANDLER, address, (uint8_t*)(&bytes), count, VL53L0X_I2C_TIMEOUT) !=
           HAL_OK)
        ;
}

uint8_t readReg(uint8_t reg) {
    uint8_t val;

    if ((status = HAL_I2C_Mem_Read(&VL53L0X_I2C_HANDLER, address, reg, I2C_MEMADD_SIZE_8BIT, &val, 1,
                                   VL53L0X_I2C_TIMEOUT)) != HAL_OK)
        return 0;

    return val;
}

uint16_t readReg16(uint8_t reg) {
    uint8_t val[2];
    readMulti(reg, val, 2);
    return (uint16_t)val[0] << 8 | val[1];
}

uint32_t readReg32(uint8_t reg) {
    uint8_t val[4];
    readMulti(reg, val, 4);

    return (uint32_t)val[0] << 24 | (uint32_t)val[1] << 16 | (uint16_t)val[2] << 8 | val[3];
}

void readMulti(uint8_t reg, uint8_t* dst, uint8_t count) {
    HAL_I2C_Mem_Read(&VL53L0X_I2C_HANDLER, address, reg, I2C_MEMADD_SIZE_8BIT, dst, count, VL53L0X_I2C_TIMEOUT);
}

uint8_t getSpadInfo(uint8_t* count, uint8_t* type_is_aperture) {
    uint8_t temp;

    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);

    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83) | 0x04);
    writeReg(0xFF, 0x07);
    writeReg(0x81, 0x01);

    writeReg(0x80, 0x01);

    writeReg(0x94, 0x6b);
    writeReg(0x83, 0x00);
    startTimeout();
    while (readReg(0x83) == 0x00)
        if (checkTimeoutExpired())
            return 0;

    writeReg(0x83, 0x01);
    temp = readReg(0x92);

    *count = temp & 0x7F;
    *type_is_aperture = (temp >> 7) & 0x01;

    writeReg(0x81, 0x00);
    writeReg(0xFF, 0x06);
    writeReg(0x83, readReg(0x83 & ~0x04));
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x01);

    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    return 1;
}

uint8_t setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {
    uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."

    if (type == VcselPeriodPreRange) {
        // "Set phase check limits"
        switch (period_pclks) {
            case 12:
                writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                break;

            case 14:
                writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                break;

            case 16:
                writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                break;

            case 18:
                writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                break;

            default:
                // invalid period
                return 0;
        }
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

        // apply new VCSEL period
        writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

        writeReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

        writeReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    } else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                writeReg(0xFF, 0x01);
                writeReg(ALGO_PHASECAL_LIM, 0x30);
                writeReg(0xFF, 0x00);
                break;

            case 10:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                writeReg(0xFF, 0x01);
                writeReg(ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            case 12:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                writeReg(0xFF, 0x01);
                writeReg(ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            case 14:
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                writeReg(0xFF, 0x01);
                writeReg(ALGO_PHASECAL_LIM, 0x20);
                writeReg(0xFF, 0x00);
                break;

            default:
                // invalid period
                return 0;
        }

        // apply new VCSEL period
        writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

        if (enables.pre_range)
            new_final_range_timeout_mclks += timeouts.pre_range_mclks;

        writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));

        // set_sequence_step_timeout end
    } else {
        // invalid type
        return 0;
    }

    // "Finally, the timing budget must be re-applied"

    setMeasurementTimingBudget(measurement_timing_budget_us);

    // "Perform the phase calibration. This is needed after changing on vcsel
    // period." VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(0x0);
    writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return 1;
}

uint8_t getVcselPulsePeriod(vcselPeriodType type) {
    if (type == VcselPeriodPreRange)
        return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
    else if (type == VcselPeriodFinalRange)
        return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    else
        return 255;
}

uint8_t setMeasurementTimingBudget(uint32_t budget_us) {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead = 1320;  // note that this is different than the value in get_
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget)
        return 0;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

    if (enables.dss)
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc)
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

    if (enables.pre_range)
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us) {
            // "Requested timeout too big."
            return 0;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
            final_range_timeout_mclks += timeouts.pre_range_mclks;

        writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));

        // set_sequence_step_timeout() end

        measurement_timing_budget_us = budget_us;  // store for internal reuse
    }
    return 1;
}

uint32_t getMeasurementTimingBudget() {
    SequenceStepEnables enables;
    SequenceStepTimeouts timeouts;

    const uint16_t StartOverhead = 1910;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(&enables);
    getSequenceStepTimeouts(&enables, &timeouts);

    if (enables.tcc)
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

    if (enables.dss)
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    else if (enables.msrc)
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

    if (enables.pre_range)
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);

    if (enables.final_range)
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);

    measurement_timing_budget_us = budget_us;  // store for internal reuse
    return budget_us;
}

void getSequenceStepEnables(SequenceStepEnables* enables) {
    uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

    enables->tcc = (sequence_config >> 4) & 0x1;
    enables->dss = (sequence_config >> 3) & 0x1;
    enables->msrc = (sequence_config >> 2) & 0x1;
    enables->pre_range = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(SequenceStepEnables* enables, SequenceStepTimeouts* timeouts) {
    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);
    timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us =
        timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
    timeouts->pre_range_mclks = decodeTimeout(readReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us =
        timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);
    timeouts->final_range_mclks = decodeTimeout(readReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;

    timeouts->final_range_us =
        timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
}

uint16_t decodeTimeout(uint16_t reg_val) {
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t encodeTimeout(uint16_t timeout_mclks) {
    // format: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    } else {
        return 0;
    }
}

uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint8_t performSingleRefCalibration(uint8_t vhv_init_byte) {
    writeReg(SYSRANGE_START,
             0x01 | vhv_init_byte);  // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout();
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        if (checkTimeoutExpired())
            return 0;

    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(SYSRANGE_START, 0x00);

    return 1;
}
