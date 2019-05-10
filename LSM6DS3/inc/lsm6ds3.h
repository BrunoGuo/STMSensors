/**
 * @file lsm6ds3.h
 *
 * @author Daniel Nery <daniel.nery@thunderatz.org>
 *
 * Equipe ThundeRatz de Robotica
 *
 * @date 10/2017
 *
 * @see https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
 */

#ifndef _LSM6DS3_H_
#define _LSM6DS3_H_

#include <stdint.h>
#include <stdbool.h>

#include "i2c.h"

/*****************************************
 * Public Constants
 *****************************************/

#define LSM6DS3_DEFAULT_ADDRESS 0xD6

/*****************************************
 * Public Types
 *****************************************/

// Return values
typedef enum lsm6ds3_status {
    IMU_SUCCESS,
    IMU_HW_ERROR,
    IMU_NOT_SUPPORTED,
    IMU_GENERIC_ERROR,
    IMU_OUT_OF_BOUNDS,
    IMU_ALL_ONES_WARNING
} lsm6ds3_status_t;

typedef struct __attribute__((packed)) lsm6ds3_settings {
    // Gyro settings
    bool gyroEnabled;
    uint16_t gyroRange;
    uint16_t gyroSampleRate;
    uint16_t gyroBandWidth;

    bool gyroFifoEnabled;
    uint8_t gyroFifoDecimation;

    // Accelerometer settings
    bool accelEnabled;
    bool accelODROff;
    uint16_t accelRange;
    uint16_t accelSampleRate;
    uint16_t accelBandWidth;

    bool accelFifoEnabled;
    uint8_t accelFifoDecimation;

    // Temperature settings
    bool tempEnabled;

    // Non-basic mode settings
    uint8_t commMode;

    // FIFO control data
    uint16_t fifoThreshold;
    int16_t fifoSampleRate;
    uint8_t fifoModeWord;
} lsm6ds3_settings_t;

typedef struct __attribute__((packed)) lsm6ds3 {
    uint8_t addr;
    I2C_HandleTypeDef* hi2c;
    lsm6ds3_settings_t settings;
} lsm6ds3_t;

/*****************************************
 * Public Function Prototypes
 *****************************************/

lsm6ds3_settings_t lsm6ds3_get_default_settings(void);
lsm6ds3_status_t lsm6ds3_init(lsm6ds3_t* lsm6ds3);

int16_t lsm6ds3_readRawAccelX(lsm6ds3_t* lsm6ds3);
int16_t lsm6ds3_readRawAccelY(lsm6ds3_t* lsm6ds3);
int16_t lsm6ds3_readRawAccelZ(lsm6ds3_t* lsm6ds3);
int16_t lsm6ds3_readRawGyroX(lsm6ds3_t* lsm6ds3);
int16_t lsm6ds3_readRawGyroY(lsm6ds3_t* lsm6ds3);
int16_t lsm6ds3_readRawGyroZ(lsm6ds3_t* lsm6ds3);

float lsm6ds3_readFloatAccelX(lsm6ds3_t* lsm6ds3);
float lsm6ds3_readFloatAccelY(lsm6ds3_t* lsm6ds3);
float lsm6ds3_readFloatAccelZ(lsm6ds3_t* lsm6ds3);
float lsm6ds3_readFloatGyroX(lsm6ds3_t* lsm6ds3);
float lsm6ds3_readFloatGyroY(lsm6ds3_t* lsm6ds3);
float lsm6ds3_readFloatGyroZ(lsm6ds3_t* lsm6ds3);

#endif
