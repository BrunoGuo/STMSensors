/**
 * Arquivo: lsm6ds3.c
 *
 * Autor: Daniel Nery Silva de Oliveira
 *
 * Equipe ThundeRatz de Robotica
 * 10/2017
 */

//! Ver: https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

#include "lsm6ds3.h"

#define TIMEOUT 1000

// Public variables
sensor_settings_t lsm6ds3_settings;

// Private variables
static uint8_t address = DEFAULT_ADDRESS;
static HAL_StatusTypeDef status;

// Private functions
static void writeReg(uint8_t reg, uint8_t val);
static void writeReg16(uint8_t reg, uint16_t val);
static void writeReg32(uint8_t reg, uint32_t val);
static void writeMulti(uint8_t reg, uint8_t* src, uint8_t count);

static uint8_t readReg(uint8_t reg);
static uint16_t readReg16(uint8_t reg);
static uint32_t readReg32(uint8_t reg);
static void readMulti(uint8_t reg, uint8_t* dst, uint8_t count);

// Public functions
status_t lsm6ds3_init() {
    status_t returnStatus = IMU_SUCCESS;

    // Check ready
    HAL_Delay(10);

    if(readReg(LSM6DS3_ACC_GYRO_WHO_AM_I_REG) != 0x69)
        returnStatus = IMU_HW_ERROR;

    // Settings
    lsm6ds3_settings.gyroEnabled         = 1;
    lsm6ds3_settings.gyroRange           = 2000;
    lsm6ds3_settings.gyroSampleRate      = 416;
    lsm6ds3_settings.gyroBandWidth       = 400;
    lsm6ds3_settings.gyroFifoEnabled     = 1;
    lsm6ds3_settings.gyroFifoDecimation  = 1;

    lsm6ds3_settings.accelEnabled        = 1;
    lsm6ds3_settings.accelODROff         = 1;
    lsm6ds3_settings.accelRange          = 16;
    lsm6ds3_settings.accelSampleRate     = 416;
    lsm6ds3_settings.accelBandWidth      = 100;
    lsm6ds3_settings.accelFifoEnabled    = 1;
    lsm6ds3_settings.accelFifoDecimation = 1;

    lsm6ds3_settings.tempEnabled = 1;

    lsm6ds3_settings.commMode = 1;

    lsm6ds3_settings.fifoThreshold  = 3000;
    lsm6ds3_settings.fifoSampleRate = 10;
    lsm6ds3_settings.fifoModeWord   = 0;


    uint8_t data_to_write = 0;

    // Accel Setup
    if (lsm6ds3_settings.accelEnabled) {
        switch (lsm6ds3_settings.accelBandWidth) {
            case 50:
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_50Hz; break;

            case 100:
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_100Hz; break;

            case 200:
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_200Hz; break;

            case 400:
            default:
                data_to_write |= LSM6DS3_ACC_GYRO_BW_XL_400Hz; break;
        }

        switch (lsm6ds3_settings.accelRange) {
            case 2:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_2g; break;

            case 4:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_4g; break;

            case 8:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_8g; break;

            case 16:
            default:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_XL_16g; break;
        }

        switch (lsm6ds3_settings.accelSampleRate) {
            case 13:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_13Hz; break;

            case 26:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz; break;

            case 52:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_52Hz; break;

            case 104:
            default:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz; break;

            case 208:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz; break;

            case 416:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_416Hz; break;

            case 833:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_833Hz; break;

            case 1660:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_1660Hz; break;

            case 3330:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_3330Hz; break;

            case 6660:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_6660Hz; break;

            case 13330:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_XL_13330Hz; break;
        }
    }

    writeReg(LSM6DS3_ACC_GYRO_CTRL1_XL, data_to_write);

    data_to_write = readReg(LSM6DS3_ACC_GYRO_CTRL4_C);
    data_to_write &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
    if (lsm6ds3_settings.accelODROff)
        data_to_write |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;

    writeReg(LSM6DS3_ACC_GYRO_CTRL4_C, data_to_write);

    // Setup gyro
    data_to_write = 0;
    if (lsm6ds3_settings.gyroEnabled) {
        switch (lsm6ds3_settings.gyroRange) {
            case 125:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_125_ENABLED; break;

            case 245:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_245dps; break;

            case 500:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_500dps; break;

            case 1000:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_1000dps; break;

            case 2000:
            default:
                data_to_write |= LSM6DS3_ACC_GYRO_FS_G_2000dps; break;
        }

        switch (lsm6ds3_settings.gyroSampleRate) {
            case 13:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_13Hz; break;

            case 26:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_26Hz; break;

            case 52:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_52Hz; break;

            case 104:
            default:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_104Hz; break;

            case 208:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_208Hz; break;

            case 416:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_416Hz; break;

            case 833:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_833Hz; break;

            case 1660:
                data_to_write |= LSM6DS3_ACC_GYRO_ODR_G_1660Hz; break;
        }
    }

    writeReg(LSM6DS3_ACC_GYRO_CTRL2_G, data_to_write);

    return returnStatus;
}

int16_t lsm6ds3_readRawAccelX() {
    return readReg16(LSM6DS3_ACC_GYRO_OUTX_L_XL);
}

int16_t lsm6ds3_readRawAccelY() {
    return readReg16(LSM6DS3_ACC_GYRO_OUTY_L_XL);
}

int16_t lsm6ds3_readRawAccelZ() {
    return readReg16(LSM6DS3_ACC_GYRO_OUTZ_L_XL);
}

int16_t lsm6ds3_readRawGyroX() {
    return readReg16(LSM6DS3_ACC_GYRO_OUTX_L_G);
}

int16_t lsm6ds3_readRawGyroY() {
    return readReg16(LSM6DS3_ACC_GYRO_OUTY_L_G);
}

int16_t lsm6ds3_readRawGyroZ() {
    return readReg16(LSM6DS3_ACC_GYRO_OUTZ_L_G);
}

#define calcAccel(acc) ((float)(acc) * 0.061 * (lsm6ds3_settings.accelRange >> 1) / 1000.0)

float lsm6ds3_readFloatAccelX() {
    return calcAccel(lsm6ds3_readRawAccelX());
}

float lsm6ds3_readFloatAccelY() {
    return calcAccel(lsm6ds3_readRawAccelY());
}

float lsm6ds3_readFloatAccelZ() {
    return calcAccel(lsm6ds3_readRawAccelZ());
}

#define calcGyro(gyr) ((float)(gyr) * 4.375 * (lsm6ds3_settings.gyroRange == 245 ? 2 : lsm6ds3_settings.gyroRange / 125) / 1000.0)

float lsm6ds3_readFloatGyroX() {
    return calcGyro(lsm6ds3_readRawGyroX());
}

float lsm6ds3_readFloatGyroY() {
    return calcGyro(lsm6ds3_readRawGyroY());
}

float lsm6ds3_readFloatGyroZ() {
    return calcGyro(lsm6ds3_readRawGyroZ());
}

// Private functions
void writeReg(uint8_t reg, uint8_t val) {
    uint8_t bytes[2] = {
        reg,
        val
    };

    while(HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t*)(&bytes), 2, TIMEOUT) != HAL_OK);
}

void writeReg16(uint8_t reg, uint16_t val) {
    uint8_t bytes[3] = {
        reg,
        (val >> 8) & 0xFF,
         val       & 0xFF
    };

    while(HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t*)(&bytes), 3, TIMEOUT) != HAL_OK);
}

void writeReg32(uint8_t reg, uint32_t val) {
    uint8_t bytes[5] = {
        reg,
        (val >> 24) & 0xFF,
        (val >> 16) & 0xFF,
        (val >>  8) & 0xFF,
         val        & 0xFF
    };

    while(HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t*)(&bytes), 5, TIMEOUT) != HAL_OK);
}

void writeMulti(uint8_t reg, uint8_t* src, uint8_t count) {
    if (count > 31)
        return;

    count++;
    uint8_t bytes[32] = { 0	};
    bytes[0] = reg;
    for (uint8_t i = 1; i < count; i++)
        bytes[i] = src[i-1];

    while(HAL_I2C_Master_Transmit(&hi2c1, address, (uint8_t*)(&bytes), count, TIMEOUT) != HAL_OK);
}

uint8_t readReg(uint8_t reg) {
    uint8_t val;

    if((status = HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, TIMEOUT)) != HAL_OK)
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

    return (uint32_t)val[0] << 24 |
           (uint32_t)val[1] << 16 |
           (uint16_t)val[2] <<  8 |
                        val[3];
}

void readMulti(uint8_t reg, uint8_t* dst, uint8_t count) {
    HAL_I2C_Mem_Read(&hi2c1, address, reg, I2C_MEMADD_SIZE_8BIT, dst, count, TIMEOUT);
}
