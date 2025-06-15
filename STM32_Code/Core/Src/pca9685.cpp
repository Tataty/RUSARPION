#include "pca9685.h"

#include <math.h>
#include <assert.h>
#include <string.h>

#define PCA9685_SET_BIT_MASK(BYTE, MASK)      ((BYTE) |= (uint8_t)(MASK))
#define PCA9685_CLEAR_BIT_MASK(BYTE, MASK)    ((BYTE) &= (uint8_t)(~(uint8_t)(MASK)))
#define PCA9685_READ_BIT_MASK(BYTE, MASK)     ((BYTE) & (uint8_t)(MASK))

/**
 * Registers addresses.
 */
typedef enum
{
	PCA9685_REGISTER_MODE1 = 0x00,
	PCA9685_REGISTER_MODE2 = 0x01,
	PCA9685_REGISTER_LED0_ON_L = 0x06,
	PCA9685_REGISTER_ALL_LED_ON_L = 0xfa,
	PCA9685_REGISTER_ALL_LED_ON_H = 0xfb,
	PCA9685_REGISTER_ALL_LED_OFF_L = 0xfc,
	PCA9685_REGISTER_ALL_LED_OFF_H = 0xfd,
	PCA9685_REGISTER_PRESCALER = 0xfe
} pca9685_register_t;

/**
 * Bit masks for the mode 1 register.
 */
typedef enum
{
	PCA9685_REGISTER_MODE1_SLEEP = (1u << 4u),
	PCA9685_REGISTER_MODE1_RESTART = (1u << 7u)
} pca9685_register_mode1_t;

/**
 * Logarithmic dimming table, mapping from 255 inputs to 12 bit PWM values.
 */

static bool pca9685_write_u8(pca9685_handle_t *handle, uint8_t address, uint8_t value)
{
	uint8_t data[] = {address, value};
	return HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, data, 2, PCA9685_I2C_TIMEOUT) == HAL_OK;
}

static bool pca9685_write_data(pca9685_handle_t *handle, uint8_t address, uint8_t *data, size_t length)
{
    if (length == 0 || length > 4) {
        return false;
    }

    uint8_t transfer[5];
    transfer[0] = address;

    memcpy(&transfer[1], data, length);

    return HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, transfer, length + 1, PCA9685_I2C_TIMEOUT) == HAL_OK;
}

static bool pca9685_read_u8(pca9685_handle_t *handle, uint8_t address, uint8_t *dest)
{
	if (HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, &address, 1, PCA9685_I2C_TIMEOUT) != HAL_OK) {
		return false;
	}

	return HAL_I2C_Master_Receive(handle->i2c_handle, handle->device_address, dest, 1, PCA9685_I2C_TIMEOUT) == HAL_OK;
}

bool pca9685_init(pca9685_handle_t *handle)
{
	assert(handle->i2c_handle != NULL);

	bool success = true;

	// Set mode registers to default values (Auto-Increment, Sleep, Open-Drain).
	uint8_t mode1_reg_default_value = 0b00110000u;
	uint8_t mode2_reg_default_value = 0b00000100u;

	if (handle->inverted) {
		mode2_reg_default_value |= 0b00010100u;
	}

	success &= pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg_default_value);
	success &= pca9685_write_u8(handle, PCA9685_REGISTER_MODE2, mode2_reg_default_value);

    // Turn all channels off to begin with.
    //uint8_t data[4] = { 0x00, 0x00, 0x00, 0x10 };
    //success &= pca9685_write_data(handle, PCA9685_REGISTER_ALL_LED_ON_L, data, 4);

	success &= pca9685_set_pwm_frequency(handle, 50);
	success &= pca9685_wakeup(handle);

	return success;
}

bool pca9685_is_sleeping(pca9685_handle_t *handle, bool *sleeping)
{
	bool success = true;

	// Read the current state of the mode 1 register.
	uint8_t mode1_reg;
	success &= pca9685_read_u8(handle, PCA9685_REGISTER_MODE1, &mode1_reg);

	// Check if the sleeping bit is set.
	*sleeping = PCA9685_READ_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);

	return success;
}

bool pca9685_sleep(pca9685_handle_t *handle)
{
	bool success = true;

	// Read the current state of the mode 1 register.
	uint8_t mode1_reg;
	success &= pca9685_read_u8(handle, PCA9685_REGISTER_MODE1, &mode1_reg);

	// Don't write the restart bit back and set the sleep bit.
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
	PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);
	success &= pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg);

	return success;
}

bool pca9685_wakeup(pca9685_handle_t *handle)
{
	bool success = true;

	// Read the current state of the mode 1 register.
	uint8_t mode1_reg;
	success &= pca9685_read_u8(handle, PCA9685_REGISTER_MODE1, &mode1_reg);

	bool restart_required = PCA9685_READ_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);

	// Clear the restart bit for now and clear the sleep bit.
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);
	success &= pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg);

	if (restart_required) {

		// Oscillator requires at least 500us to stabilise, so wait 1ms.
		HAL_Delay(1);

		PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
		success &= pca9685_write_u8(handle, PCA9685_REGISTER_MODE1, mode1_reg);
	}

	return success;
}

bool pca9685_set_pwm_frequency(pca9685_handle_t *handle, float frequency)
{
	assert(frequency >= 24);
	assert(frequency <= 1526);

	bool success = true;

	// Calculate the prescaler value (see datasheet page 25)
	uint8_t prescaler = (uint8_t)roundf(25000000.0f / (4096 * frequency)) - 1;

	bool already_sleeping;
	success &= pca9685_is_sleeping(handle, &already_sleeping);

	// The prescaler can only be changed in sleep mode.
	if (!already_sleeping) {
		success &= pca9685_sleep(handle);
	}

	// Write the new prescaler value.
	success &= pca9685_write_u8(handle, PCA9685_REGISTER_PRESCALER, prescaler);

	// If the device wasn't sleeping, return from sleep mode.
	if (!already_sleeping) {
		success &= pca9685_wakeup(handle);
	}

	return success;
}

bool pca9685_set_channel_pwm_times(pca9685_handle_t *handle, uint8_t channel, uint16_t off_time)
{
	assert(channel >= 0);
	assert(channel < 16);

	assert(off_time >= 0);
	assert(off_time <= 4096);

	uint8_t data[4] = { 0, 0, off_time, off_time >> 8u };
	return pca9685_write_data(handle, PCA9685_REGISTER_LED0_ON_L + channel * 4, data, 4);
}
