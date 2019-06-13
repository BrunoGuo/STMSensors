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
#include <stdbool.h>

#include "gpio.h"
#include "i2c.h"

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L0X_DEFAULT_ADDRESS (0x52)        /**< Default sensor address, don't change */
#define VL53L0X_TIMEOUT_RETURN_VALUE (0XFFFF) /**< Value returned if getRange times out */

/*****************************************
 * Public Types
 *****************************************/

/**
 * @brief Hold configuration of a single VL53L0X sensor.
 */
typedef struct __attribute__((packed)) vl53l0x_handler {
    uint8_t            addr;             /**< Sensor I2C device address, should be initialized as DEFAULT */
    I2C_HandleTypeDef* hi2c;             /**< Pointer to sensor I2C handler */

    GPIO_TypeDef*      xshut_port;       /**< Pointer to sensor xshut GPIO handler */
    uint16_t           xshut_pin;        /**< Sensor xshut pin number */

    uint8_t            stop_variable;    /**< Internal variable */
} vl53l0x_handler_t;

/*****************************************
 * Public Function Prototypes
 *****************************************/

/**
 * @brief Initialize a single VL53L0X device.
 *
 * @param vl53l0x Sensor to be initialized
 *
 * @note This must be called before any other function. Be sure only one
 *       uninitialized sensor is active through XSHUT pin.
 *
 * @return false on failure, true on success.
 */
bool vl53l0x_init(vl53l0x_t* vl53l0x);

/**
 * @brief Turn off xshut pin of a given sensor.
 *
 * @param vl53l0x Sensor to turn off the xshut pin
 */
void vl53l0x_xshut_off(vl53l0x_t* vl53l0x);

/**
 * @brief Turn on xshut pin of a given sensor.
 *
 * @param vl53l0x Sensor to turn on the xshut pin
 */
void vl53l0x_xshut_on(vl53l0x_t* vl53l0x);

/**
 * @brief Change I2C device address of a given sensor.
 *
 * @param vl53l0x Sensor to change the I2C device address
 * @param new_addr 8 bit address to be set
 */
void vl53l0x_set_dev_address(vl53l0x_t* vl53l0x, uint8_t new_addr);

/**
 * @brief Start continuous measuring of a given sensor.
 *
 * @param vl53l0x Sensor to start the continuous measuring
 * @param period_ms Measuring period
 */
void vl53l0x_start_continuous(vl53l0x_t* vl53l0x, uint32_t period_ms);

/**
 * @brief Stop continuous measuring of a given sensor.
 *
 * @param vl53l0x Sensor to stop the continuous measuring
 */
void vl53l0x_stop_continuous(vl53l0x_t* vl53l0x);

/**
 * @brief Get the last measured range of a given sensor.
 *
 * @param vl53l0x Sensor to get the last measured range.
 *
 * @note vl53l0x_start_continuous must be called first.
 *
 * @return Range in millimeters or VL53L0X_TIMEOUT_RETURN_VALUE on failure.
 */
uint16_t vl53l0x_get_range(vl53l0x_t* vl53l0x);

#endif // __VL53L0X_H__
