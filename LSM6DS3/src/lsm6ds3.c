/**
 * @file lsm6ds3.c
 *
 * @author Daniel Nery <daniel.nery@thunderatz.org>
 *
 * Equipe ThundeRatz de Robotica
 *
 * @date 10/2017
 *
 * @see https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
 */

#include "lsm6ds3.h"
#include "lsm6ds3_registers.h"

/*****************************************
 * Private Constants
 *****************************************/

#define LSM6DS3_I2C_TIMEOUT_MS 1000

/*****************************************
 * Private Variables
 *****************************************/

static HAL_StatusTypeDef status;

/*****************************************
 * Private Function Prototypes
 *****************************************/

/**
 * @brief HAL I2C wrapper functions.
 */
static void write_reg(lsm6ds3_t* lsm6ds3, uint8_t reg, uint8_t val);
static void write_reg16(lsm6ds3_t* lsm6ds3, uint8_t reg, uint16_t val);
static void write_reg32(lsm6ds3_t* lsm6ds3, uint8_t reg, uint32_t val);
static void write_multi(lsm6ds3_t* lsm6ds3, uint8_t reg, uint8_t* src, uint8_t count);
static uint8_t read_reg(lsm6ds3_t* lsm6ds3, uint8_t reg);
static uint16_t read_reg16(lsm6ds3_t* lsm6ds3, uint8_t reg);
static uint32_t read_reg32(lsm6ds3_t* lsm6ds3, uint8_t reg);
static void read_multi(lsm6ds3_t* lsm6ds3, uint8_t reg, uint8_t* dst, uint8_t count);

/*****************************************
 * Public Function Body Definitions
 *****************************************/

lsm6ds3_settings_t lsm6ds3_get_default_settings(void) {
    return (lsm6ds3_settings_t){
        .gyroEnabled = true,
        .gyroRange = 2000,
        .gyroSampleRate = 416,
        .gyroBandWidth = 400,
        .gyroFifoEnabled = true,
        .gyroFifoDecimation = 1,

        .accelEnabled = true,
        .accelODROff = true,
        .accelRange = 16,
        .accelSampleRate = 416,
        .accelBandWidth = 100,
        .accelFifoEnabled = true,
        .accelFifoDecimation = 1,

        .tempEnabled = true,

        .commMode = 1,

        .fifoThreshold = 3000,
        .fifoSampleRate = 10,
        .fifoModeWord = 0,
    };
}

lsm6ds3_status_t lsm6ds3_init(lsm6ds3_t* lsm6ds3) {
    lsm6ds3_status_t returnStatus = IMU_SUCCESS;

    // Check ready
    HAL_Delay(10);

    if (read_reg(lsm6ds3, LSM6DS3_ACC_GYRO_WHO_AM_I_REG) != 0x69) {
        returnStatus = IMU_HW_ERROR;
    }

    uint8_t data_to_write = 0;

    // Accel Setup
    if (lsm6ds3->settings.accelEnabled) {
        switch (lsm6ds3->settings.accelBandWidth) {
            case 50: {
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_50Hz;
                break;
            }

            case 100: {
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
                break;
            }

            case 200: {
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
                break;
            }

            case 400:
            default: {
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_400Hz;
                break;
            }
        }

        switch (lsm6ds3->settings.accelRange) {
            case 2: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_2g;
                break;
            }

            case 4: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_4g;
                break;
            }

            case 8: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_8g;
                break;
            }

            case 16:
            default: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_16g;
                break;
            }
        }

        switch (lsm6ds3->settings.accelSampleRate) {
            case 13: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz;
                break;
            }

            case 26: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;
                break;
            }

            case 52: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz;
                break;
            }

            case 104:
            default: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
                break;
            }

            case 208: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
                break;
            }

            case 416: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz;
                break;
            }

            case 833: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz;
                break;
            }

            case 1660: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz;
                break;
            }

            case 3330: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz;
                break;
            }

            case 6660: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz;
                break;
            }

            case 13330: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz;
                break;
            }
        }
    }

    write_reg(lsm6ds3, LSM6DS3_ACC_GYRO_CTRL1_XL, data_to_write);

    data_to_write = read_reg(lsm6ds3, LSM6DS3_ACC_GYRO_CTRL4_C);
    data_to_write &= ~((uint8_t) LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

    if (lsm6ds3->settings.accelODROff) {
        data_to_write |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
    }

    write_reg(lsm6ds3, LSM6DS3_ACC_GYRO_CTRL4_C, data_to_write);

    // Setup gyro
    data_to_write = 0;
    if (lsm6ds3->settings.gyroEnabled) {
        switch (lsm6ds3->settings.gyroRange) {
            case 125: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_125_ENABLED;
                break;
            }

            case 245: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_245dps;
                break;
            }

            case 500: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_500dps;
                break;
            }

            case 1000: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_1000dps;
                break;
            }

            case 2000:
            default: {
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
                break;
            }
        }

        switch (lsm6ds3->settings.gyroSampleRate) {
            case 13: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_13Hz;
                break;
            }

            case 26: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_26Hz;
                break;
            }

            case 52: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_52Hz;
                break;
            }

            case 104:
            default: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_104Hz;
                break;
            }

            case 208: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
                break;
            }

            case 416: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_416Hz;
                break;
            }

            case 833: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_833Hz;
                break;
            }

            case 1660: {
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz;
                break;
            }
        }
    }

    write_reg(lsm6ds3, LSM6DS3_ACC_GYRO_CTRL2_G, data_to_write);

    return returnStatus;
}

int16_t lsm6ds3_readRawAccelX(lsm6ds3_t* lsm6ds3) {
    return read_reg16(lsm6ds3, LSM6DS3_ACC_GYRO_OUTX_L_XL);
}

int16_t lsm6ds3_readRawAccelY(lsm6ds3_t* lsm6ds3) {
    return read_reg16(lsm6ds3, LSM6DS3_ACC_GYRO_OUTY_L_XL);
}

int16_t lsm6ds3_readRawAccelZ(lsm6ds3_t* lsm6ds3) {
    return read_reg16(lsm6ds3, LSM6DS3_ACC_GYRO_OUTZ_L_XL);
}

int16_t lsm6ds3_readRawGyroX(lsm6ds3_t* lsm6ds3) {
    return read_reg16(lsm6ds3, LSM6DS3_ACC_GYRO_OUTX_L_G);
}

int16_t lsm6ds3_readRawGyroY(lsm6ds3_t* lsm6ds3) {
    return read_reg16(lsm6ds3, LSM6DS3_ACC_GYRO_OUTY_L_G);
}

int16_t lsm6ds3_readRawGyroZ(lsm6ds3_t* lsm6ds3) {
    return read_reg16(lsm6ds3, LSM6DS3_ACC_GYRO_OUTZ_L_G);
}

#define calcAccel(acc) ((float) (acc) *0.061 * (lsm6ds3->settings.accelRange >> 1) / 1000.0)

float lsm6ds3_readFloatAccelX(lsm6ds3_t* lsm6ds3) {
    return calcAccel(lsm6ds3_readRawAccelX(lsm6ds3));
}

float lsm6ds3_readFloatAccelY(lsm6ds3_t* lsm6ds3) {
    return calcAccel(lsm6ds3_readRawAccelY(lsm6ds3));
}

float lsm6ds3_readFloatAccelZ(lsm6ds3_t* lsm6ds3) {
    return calcAccel(lsm6ds3_readRawAccelZ(lsm6ds3));
}

#define calcGyro(gyr) \
    ((float) (gyr) *4.375 * (lsm6ds3->settings.gyroRange == 245 ? 2 : lsm6ds3->settings.gyroRange / 125) / 1000.0)

float lsm6ds3_readFloatGyroX(lsm6ds3_t* lsm6ds3) {
    return calcGyro(lsm6ds3_readRawGyroX(lsm6ds3));
}

float lsm6ds3_readFloatGyroY(lsm6ds3_t* lsm6ds3) {
    return calcGyro(lsm6ds3_readRawGyroY(lsm6ds3));
}

float lsm6ds3_readFloatGyroZ(lsm6ds3_t* lsm6ds3) {
    return calcGyro(lsm6ds3_readRawGyroZ(lsm6ds3));
}

// Private functions
void write_reg(lsm6ds3_t* lsm6ds3, uint8_t reg, uint8_t val) {
    uint8_t bytes[2] = {reg, val};
    uint8_t status;

    uint32_t tickstart = HAL_GetTick();
    do {
        status = HAL_I2C_Master_Transmit(lsm6ds3->hi2c, lsm6ds3->addr, (uint8_t*) (&bytes), 2, LSM6DS3_I2C_TIMEOUT_MS);
    } while (status != HAL_OK || ((HAL_GetTick() - tickstart) > LSM6DS3_I2C_TIMEOUT_MS));
}

void write_reg16(lsm6ds3_t* lsm6ds3, uint8_t reg, uint16_t val) {
    uint8_t bytes[3] = {reg, (val >> 8) & 0xFF, val & 0xFF};
    uint8_t status;

    uint32_t tickstart = HAL_GetTick();
    do {
        status = HAL_I2C_Master_Transmit(lsm6ds3->hi2c, lsm6ds3->addr, (uint8_t*) (&bytes), 3, LSM6DS3_I2C_TIMEOUT_MS);
    } while (status != HAL_OK || ((HAL_GetTick() - tickstart) > LSM6DS3_I2C_TIMEOUT_MS));
}

void write_reg32(lsm6ds3_t* lsm6ds3, uint8_t reg, uint32_t val) {
    uint8_t bytes[5] = {reg, (val >> 24) & 0xFF, (val >> 16) & 0xFF, (val >> 8) & 0xFF, val & 0xFF};
    uint8_t status;

    uint32_t tickstart = HAL_GetTick();
    do {
        status = HAL_I2C_Master_Transmit(lsm6ds3->hi2c, lsm6ds3->addr, (uint8_t*) (&bytes), 5, LSM6DS3_I2C_TIMEOUT_MS);
    } while (status != HAL_OK || ((HAL_GetTick() - tickstart) > LSM6DS3_I2C_TIMEOUT_MS));
}

void write_multi(lsm6ds3_t* lsm6ds3, uint8_t reg, uint8_t* src, uint8_t count) {
    if (count > 31) {
        return;
    }

    count++;
    uint8_t bytes[32] = {0};
    bytes[0] = reg;
    for (uint8_t i = 1; i < count; i++) {
        bytes[i] = src[i - 1];
    }

    uint8_t status;
    uint32_t tickstart = HAL_GetTick();
    do {
        status = HAL_I2C_Master_Transmit(lsm6ds3->hi2c, lsm6ds3->addr, (uint8_t*) (&bytes), count, LSM6DS3_I2C_TIMEOUT_MS);
    } while (status != HAL_OK || ((HAL_GetTick() - tickstart) > LSM6DS3_I2C_TIMEOUT_MS));
}

uint8_t read_reg(lsm6ds3_t* lsm6ds3, uint8_t reg) {
    uint8_t val;

    if ((status = HAL_I2C_Mem_Read(lsm6ds3->hi2c, lsm6ds3->addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, LSM6DS3_I2C_TIMEOUT_MS)) !=
        HAL_OK) {
        return 0;
    }

    return val;
}

uint16_t read_reg16(lsm6ds3_t* lsm6ds3, uint8_t reg) {
    uint8_t val[2];
    read_multi(lsm6ds3, reg, val, 2);

    return (uint16_t) val[1] << 8 | val[0];
}

uint32_t read_reg32(lsm6ds3_t* lsm6ds3, uint8_t reg) {
    uint8_t val[4];
    read_multi(lsm6ds3, reg, val, 4);

    return (uint32_t) val[0] << 24 | (uint32_t) val[1] << 16 | (uint16_t) val[2] << 8 | val[3];
}

void read_multi(lsm6ds3_t* lsm6ds3, uint8_t reg, uint8_t* dst, uint8_t count) {
    HAL_I2C_Mem_Read(lsm6ds3->hi2c, lsm6ds3->addr, reg, I2C_MEMADD_SIZE_8BIT, dst, count, LSM6DS3_I2C_TIMEOUT_MS);
}
