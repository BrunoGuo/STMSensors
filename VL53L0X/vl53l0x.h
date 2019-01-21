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

#ifndef __VL53L0X_H__
#define __VL53L0X_H__

#include <stdint.h>

#include "i2c.h"

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L0X_DEFAULT_ADDRESS (0x52)       /**< Default sensor address, don't change */
#define VL53L0X_TIMEOUT_RETURN_VALUE (65535) /**< Value returned if getRange times out */

/*****************************************
 * Public Function Prototypes
 *****************************************/

/**
 * @brief Sets I2C handler for VL53L0X devices.
 *
 * @note This must be called first.
 */
void vl53l0x_i2c_set(I2C_HandleTypeDef* hi2c);

/**
 * @brief Initializes a single VL53L0X device.
 *
 * @note This must be called before any other function,
 *       except vl53l0x_i2c_set. Be sure only one
 *       uninitialized sensor is active through XSHUT pin.
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t vl53l0x_init();

/**
 * @brief Changes current device address.
 *
 * @param new_addr 8 bit address to be set
 */
void vl53l0x_set_dev_address(uint8_t new_addr);

/**
 * @brief Change the current working address.
 *
 * @note Subsequent functions will be applied to the sensor
 *       with the address selected here.
 *
 * @param new_addr 8 bit address to be set
 */
void vl53l0x_set_current_address(uint8_t new_addr);

/**
 * @brief Start current device continuous measuring.
 *
 * @param period_ms Measuring period
 */
void vl53l0x_start_continuous(uint32_t period_ms);

/**
 * @brief Stop current device continuous measuring.
 */
void vl53l0x_stop_continuous();

/**
 * @brief Get the current device's last measured range.
 *
 * @note vl53l0x_start_continuous must be called first.
 *
 * @return Range in millimeters or 65535 on failure.
 */
uint16_t vl53l0x_get_range();

#endif  // __VL53L0X_H__
