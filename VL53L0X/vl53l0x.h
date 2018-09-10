/**
 * @file vl53l0x.h
 *
 * @author Daniel Nery <daniel.nery@thunderatz.org>
 *
 * Equipe ThundeRatz de Robotica
 *
 * @date 03/2017
 *
 * @see https://github.com/pololu/vl53l0x-arduino
 */

#ifndef _VL53L0X_H_
#define _VL53L0X_H_

// SPECIFIC INCLUDES HERE

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L0X_DEFAULT_ADDRESS (0x52)       /**< Default sensor address, don't change */
#define VL53L0X_TIMEOUT_RETURN_VALUE (65535) /**< Value returned if getRange times out */

#define VL53L0X_I2C_HANDLER hi2c1 /**< I2C handler */

//! @see setVcselPulsePeriod
#define VL53L0X_VCSEL_PRE_RANGE 18
#define VL53L0X_VCSEL_FINAL_RANGE 14

/*****************************************
 * Public Types
 *****************************************/

/*****************************************
 * Public Function Prototypes
 *****************************************/

/**
 * @brief Initializes a single VL53L0X device.
 *
 * @note This must be called before any other function,
 *       be sure only one uninitialized sensor is active
 *       through XSHUT pin.
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t vl53l0x_init();

/**
 * @brief Changes current device address.
 *
 * @param new_addr 8 bit address to be set
 */
void vl53l0x_setDevAddress(uint8_t new_addr);

/**
 * @brief Change the current working address.
 *
 * @note Subsequent functions will be applied to the sensor
 *       with the address selected here.
 *
 * @param new_addr 8 bit address to be set
 */
void vl53l0x_setCurrentAddress(uint8_t new_addr);

/**
 * @brief Start current device continuous measuring.
 *
 * @param period_ms Measuring period
 */
void vl53l0x_startContinuous(uint32_t period_ms);

/**
 * @brief Stop current device continuous measuring.
 */
void vl53l0x_stopContinuous();

/**
 * @brief Get the current device's last measured range.
 *
 * @note vl53l0x_startContinuous must be called first.
 *
 * @return Range in millimeters or 65535 on failure.
 */
uint16_t vl53l0x_getRange();

#endif
