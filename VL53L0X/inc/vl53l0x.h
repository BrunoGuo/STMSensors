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
#include "gpio.h"

/*****************************************
 * Public Constants
 *****************************************/

#define VL53L0X_DEFAULT_ADDRESS (0x52)       /**< Default sensor address, don't change */
#define VL53L0X_TIMEOUT_RETURN_VALUE (65535) /**< Value returned if getRange times out */

/*****************************************
 * Public Types
 *****************************************/

typedef struct __attribute__((packed)) vl53l0x {
    uint8_t addr;
    I2C_HandleTypeDef* hi2c;

    GPIO_TypeDef* xshut_port;
    uint16_t xshut_pin;

    uint8_t stop_variable;
} vl53l0x_t;

/*****************************************
 * Public Function Prototypes
 *****************************************/

/**
 * @brief Initializes a single VL53L0X device.
 *
 * @note This must be called before any other function,
 *       except vl53l0x_i2c_set. Be sure only one
 *       uninitialized sensor is active through XSHUT pin.
 *
 * @return 0 on failure, 1 on success.
 */
uint8_t vl53l0x_init(vl53l0x_t* vl53l0x);

void vl53l0x_xshut_off(vl53l0x_t* vl53l0x);

void vl53l0x_xshut_on(vl53l0x_t* vl53l0x);

/**
 * @brief Changes current device address.
 *
 * @param new_addr 8 bit address to be set
 */
void vl53l0x_set_dev_address(vl53l0x_t* vl53l0x, uint8_t new_addr);

/**
 * @brief Start current device continuous measuring.
 *
 * @param period_ms Measuring period
 */
void vl53l0x_start_continuous(vl53l0x_t* vl53l0x, uint32_t period_ms);

/**
 * @brief Stop current device continuous measuring.
 */
void vl53l0x_stop_continuous(vl53l0x_t* vl53l0x);

/**
 * @brief Get the current device's last measured range.
 *
 * @note vl53l0x_start_continuous must be called first.
 *
 * @return Range in millimeters or VL53L0X_TIMEOUT_RETURN_VALUE on failure.
 */
uint16_t vl53l0x_get_range(vl53l0x_t* vl53l0x);

#endif  // __VL53L0X_H__
