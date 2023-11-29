/***************************************************************************//**
 *   @file   adp1050.c
 *   @brief  Source file for the ADP1050 Driver
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#include <stdlib.h>
#include "adp1050.h"
#include "no_os_alloc.h"
#include "no_os_delay.h"
#include "no_os_error.h"

int adp1050_send_command(struct adp1050_desc *desc, uint16_t command)
{
	uint8_t data[2];
	uint8_t command_val;

	if (command > ADP1050_EXTENDED_COMMAND) {
		data[0] = no_os_field_get(ADP1050_LSB_MASK, command);
		data[1] = no_os_field_get(ADP1050_MSB_MASK, command);

		return no_os_i2c_write(desc->i2c_desc, data, 1, 1);
	}

	command_val = no_os_field_get(ADP1050_LSB_MASK, command);

	return no_os_i2c_write(desc->i2c_desc, &command_val, 1, 1);
}

int adp1050_read(struct adp1050_desc *desc, uint16_t command, uint8_t *data,
		 uint8_t bytes_number)
{
	int ret;
	uint8_t command_val[2];

	if (command > ADP1050_EXTENDED_COMMAND) {
		command_val[0] = no_os_field_get(ADP1050_LSB_MASK, command);
		command_val[1] = no_os_field_get(ADP1050_MSB_MASK, command);

		ret = no_os_i2c_write(desc->i2c_desc, command_val, 2, 0);
		if (ret)
			return ret;
	} else {
		ret = no_os_i2c_write(desc->i2c_desc, data, 1, 0);
		if (ret)
			return ret;
	}

	return no_os_i2c_read(desc->i2c_desc, data, bytes_number, 1);
}

int adp1050_write(struct adp1050_desc *desc, uint16_t command, uint8_t *data,
		  uint8_t bytes_number)
{
	int ret;
	uint8_t command_val[2];

	if (command > ADP1050_EXTENDED_COMMAND) {
		command_val[0] = no_os_field_get(ADP1050_LSB_MASK, command);
		command_val[1] = no_os_field_get(ADP1050_MSB_MASK, command);

		ret = no_os_i2c_write(desc->i2c_desc, command_val, 2, 0);
		if (ret)
			return ret;
	} else {
		ret = no_os_i2c_write(desc->i2c_desc, data, 1, 0);
		if (ret)
			return ret;
	}

	return no_os_i2c_write(desc->i2c_desc, data, bytes_number, 1);
}

int adp1050_read_vout(struct adp1050_desc *desc, uint16_t *vout)
{
	int ret;
	uint8_t data[2];

	ret = adp1050_read(desc, ADP1050_READ_VOUT, data, 2);
	if (ret)
		return ret;

	*vout = no_os_get_unaligned_be16(data);

	return 0;
}

int adp1050_set_vout_tr(struct adp1050_desc *desc, uint16_t vout_tr)
{
	int ret;
	uint8_t data[2];

	no_os_put_unaligned_be16(vout_tr, data);

	ret = adp1050_write(desc, ADP1050_VOUT_TRANSITION_RATE, data, 2);
	if (ret)
		return ret;

	desc->vout_tran_rate = vout_tr;

	return 0;
}

int adp1050_set_command(struct adp1050_desc *desc, uint16_t command,
			uint16_t value)
{
	int ret;
	uint8_t data[2];

	no_os_put_unaligned_be16(value, data);

	return adp1050_write(desc, command, data, 2);
}

int adp1050_set_vin(struct adp1050_desc *desc, uint16_t mantissa, uint8_t exp,
		    bool state_on)
{
	uint8_t data[2];
	uint16_t val;

	if (mantissa > ADP1050_VIN_MANT_MAX || exp > ADP1050_VIN_EXP_MAX)
		return -EINVAL;

	no_os_put_unaligned_be16(mantissa, data);

	val = no_os_field_prep(ADP1050_VIN_EXP_MASK, exp) | mantissa;

	data[0] = no_os_field_get(ADP1050_LSB_MASK, val);
	data[1] = no_os_field_get(ADP1050_MSB_MASK, val);

	return adp1050_write(desc, state_on ? ADP1050_VIN_ON : ADP1050_VIN_OFF,
			     data, 2);
}

int adp1050_set_open_loop(struct adp1050_desc *desc, uint8_t rising_edge,
			  uint8_t falling_edge, enum adp1050_channel chan)
{
	int ret;

	switch (chan) {
	case ADP1050_OUTA:
		ret = adp1050_write(desc, ADP1050_OUTA_RISING_EDGE_TIMING,
				    &rising_edge, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_OUTA_FALLING_EDGE_TIMING,
				    &falling_edge, 1);
		if (ret)
			return ret;

		/* Modulation limit needs to be set to 0us. */
		ret = adp1050_write(desc, ADP1050_MODULATION_LIMIT, 0, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_OUTA_OUTB_MODULATION_SETTINGS,
				    ADP1050_OUTA_FALLING_EDGE_NEGATIVE_MOD, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_OL_OPERATION_SETTINGS,
				    ADP1050_OUTA_OL_ENABLE, 1);
		if (ret)
			return ret;

		return adp1050_write(desc, ADP1050_SOFT_START_SETTING_OL,
				     ADP1050_OL_SS_64_CYCLES, 1);
	case ADP1050_OUTB:
		ret = adp1050_write(desc, ADP1050_OUTB_RISING_EDGE_TIMING,
				    &rising_edge, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_OUTB_FALLING_EDGE_TIMING,
				    &falling_edge, 1);
		if (ret)
			return ret;

		/* Modulation limit needs to be set to 0us. */
		ret = adp1050_write(desc, ADP1050_MODULATION_LIMIT, 0, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_OUTA_OUTB_MODULATION_SETTINGS,
				    ADP1050_OUTB_FALLING_EDGE_NEGATIVE_MOD, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_OL_OPERATION_SETTINGS,
				    ADP1050_OUTB_OL_ENABLE, 1);
		if (ret)
			return ret;

		return adp1050_write(desc, ADP1050_SOFT_START_SETTING_OL,
				     ADP1050_OL_SS_64_CYCLES, 1);
	case ADP1050_SR1:
		ret = adp1050_write(desc, ADP1050_SR1_RISING_EDGE_TIMING,
				    &rising_edge, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_SR1_FALLING_EDGE_TIMING,
				    &falling_edge, 1);
		if (ret)
			return ret;

		/* Modulation limit needs to be set to 0us. */
		ret = adp1050_write(desc, ADP1050_MODULATION_LIMIT, 0, 1);
		if (ret)
			return ret;

		return adp1050_write(desc, ADP1050_OL_OPERATION_SETTINGS,
				     ADP1050_SR1_OL_ENABLE, 1);
	case ADP1050_SR2:
		ret = adp1050_write(desc, ADP1050_SR2_RISING_EDGE_TIMING,
				    &rising_edge, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_SR2_FALLING_EDGE_TIMING,
				    &falling_edge, 1);
		if (ret)
			return ret;

		/* Modulation limit needs to be set to 0us. */
		ret = adp1050_write(desc, ADP1050_MODULATION_LIMIT, 0, 1);
		if (ret)
			return ret;

		return adp1050_write(desc, ADP1050_OL_OPERATION_SETTINGS,
				     ADP1050_SR2_OL_ENABLE, 1);
	default:
		return -EINVAL;
	}
}

int adp1050_freq_sync(struct adp1050_desc *desc, bool state_on)
{
	return adp1050_write(desc, ADP1050_SYNCH_GENERAL_SETTINGS,
			     state_on ? ADP1050_FREQ_SYNC_ON : ADP1050_FREQ_SYNC_OFF, 1);
}

int adp1050_set_freq(struct adp1050_desc *desc, enum adp1050_freq freq)
{
	int ret;
	uint8_t data_freq[2];

	data_freq[0] = no_os_field_get(ADP1050_LSB_MASK, freq);
	data_freq[1] = no_os_field_get(ADP1050_MSB_MASK, freq);

	ret = adp1050_write(desc, ADP1050_FREQUENCY_SWITCH, data_freq, 2);
	if (ret)
		return ret;

	ret = adp1050_write(desc, ADP1050_GO_COMMANDS, ADP1050_FREQ_SWITCH_GO, 1);
	if (ret)
		return ret;

	return adp1050_send_command(desc, ADP1050_GO_COMMANDS);
}

int adp1050_unlock_pass(struct adp1050_desc *desc, uint16_t password,
			enum adp1050_pass_type pass_type)
{
	int ret;
	uint8_t data[2];
	uint8_t check_reg;

	data[0] = no_os_field_get(ADP1050_LSB_MASK, password);
	data[1] = no_os_field_get(ADP1050_MSB_MASK, password);

	switch (pass_type) {
	case ADP1050_CHIP_PASS:
		ret = adp1050_write(desc, ADP1050_CHIP_PASSWORD, data, 2);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_CHIP_PASSWORD, data, 2);
		if (ret)
			return ret;

		ret = adp1050_read(desc, ADP1050_FLAG_REGISTER1, &check_reg, 1);
		if (ret)
			return ret;

		return no_os_field_get(ADP1050_CHECK_CHIP_PASS_MASK, check_reg) ? 0 : -EINVAL;
	case ADP1050_EEPROM_PASS:
		if (password > ADP1050_EEPROM_DEFAULT_PASS)
			return -EINVAL;

		ret = adp1050_write(desc, ADP1050_EEPROM_PASSWORD, data, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_EEPROM_PASSWORD, data, 1);
		if (ret)
			return ret;

		ret = adp1050_read(desc, ADP1050_FLAG_REGISTER3, &check_reg, 1);
		if (ret)
			return ret;

		return no_os_field_get(ADP1050_CHECK_EEPROM_PASS_MASK, check_reg) ? 0 : -EINVAL;
	case ADP1050_TRIM_PASS:
		if (password > ADP1050_EEPROM_DEFAULT_PASS)
			return -EINVAL;

		ret = adp1050_write(desc, ADP1050_TRIM_PASSWORD, data, 1);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_TRIM_PASSWORD, data, 1);
		if (ret)
			return ret;

		return 0;
	default:
		return -EINVAL;
	}
}

int adp1050_lock_pass(struct adp1050_desc *desc, uint16_t password,
		      enum adp1050_pass_type pass_type)
{
	uint8_t data[2];

	data[1] = no_os_field_get(ADP1050_LSB_MASK, password);
	data[0] = no_os_field_get(ADP1050_MSB_MASK, password);

	switch (pass_type) {
	case ADP1050_CHIP_PASS:
		return adp1050_write(desc, ADP1050_CHIP_PASSWORD, data, 2);
	case ADP1050_EEPROM_PASS:
		return adp1050_write(desc, ADP1050_EEPROM_PASSWORD, data, 1);
	default:
		return -EINVAL;
	}
}

int adp1050_change_pass(struct adp1050_desc *desc, uint16_t old_pass,
			uint16_t new_pass, enum adp1050_pass_type pass_type)
{
	int ret;
	uint8_t data_old[2], data_new[2];

	if (pass_type == ADP1050_EEPROM_PASS && new_pass > ADP1050_EEPROM_DEFAULT_PASS)
		return -EINVAL;

	data_old[0] = no_os_field_get(ADP1050_LSB_MASK, old_pass);
	data_old[1] = no_os_field_get(ADP1050_MSB_MASK, old_pass);

	data_new[0] = no_os_field_get(ADP1050_LSB_MASK, new_pass);
	data_new[1] = no_os_field_get(ADP1050_MSB_MASK, new_pass);

	switch (pass_type) {
	case ADP1050_CHIP_PASS:
		ret = adp1050_write(desc, ADP1050_CHIP_PASSWORD, data_old, 2);
		if (ret)
			return ret;

		ret = adp1050_write(desc, ADP1050_CHIP_PASSWORD, data_new, 2);
		if (ret)
			return ret;

		return adp1050_send_command(desc, ADP1050_STORE_USER_ALL);
	case ADP1050_EEPROM_PASS:
		ret = adp1050_write(desc, ADP1050_EEPROM_PASSWORD, data_old, 1);
		if (ret)
			return ret;

		return adp1050_write(desc, ADP1050_EEPROM_PASSWORD, data_new, 1);
	default:
		return -EINVAL;
	}
}

int adp1050_software_reset(struct adp1050_desc *desc)
{
	int ret;

	ret = adp1050_write(desc, ADP1050_SOFTWARE_RESET_SETTINGS,
			    ADP1050_SW_RES_DELAY_500MS
			    | ADP1050_SW_RES_NO_DELAY, 1);
	if (ret)
		return ret;

	ret = adp1050_write(desc, ADP1050_SOFTWARE_RESET_GO,
			    ADP1050_SOFTWARE_RESET_ON, 1);
	if (ret)
		return ret;

	return adp1050_send_command(desc, ADP1050_SOFTWARE_RESET_GO);
}

int adp1050_init(struct adp1050_desc **desc,
		 struct adp1050_init_param *init_param)
{
	struct adp1050_desc *descriptor;
	int ret;

	descriptor = (struct adp1050_desc *)no_os_calloc(sizeof(*descriptor), 1);
	if (!descriptor)
		return -ENOMEM;

	ret = no_os_i2c_init(&descriptor->i2c_desc, init_param->i2c_param);
	if (ret) {
		no_os_free(descriptor);
		return ret;
	}

	ret = no_os_gpio_get(&descriptor->pg_alt_desc, init_param->pg_alt_param);
	if (ret)
		return ret;

	ret = no_os_gpio_direction_output(descriptor->pg_alt_desc,
					  NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	ret = adp1050_write(descriptor, ADP1050_ON_OFF_CONFIG,
			    &init_param->on_off_config, 1);
	if (ret)
		return ret;

	ret = adp1050_write(descriptor, ADP1050_OPERATION, ADP1050_OPERATION_ON, 1);
	if (ret)
		return ret;

	ret = adp1050_send_command(descriptor, ADP1050_OPERATION);
	if (ret)
		return ret;

	ret = adp1050_write(descriptor, ADP1050_TON_DELAY,
			    ADP1050_TON_DELAY_10MS, 1);
	if (ret)
		return ret;

	ret = adp1050_write(descriptor, ADP1050_TON_RISE,
			    ADP1050_TON_RISE_1750US, 1);
	if (ret)
		return ret;

	ret = no_os_gpio_set_value(descriptor->pg_alt_desc, NO_OS_GPIO_HIGH);
	if (ret)
		return ret;

	*desc = descriptor;

	return 0;
}

int adp1050_remove(struct adp1050_desc *desc)
{
	int ret;

	ret = adp1050_write(desc, ADP1050_TOFF_DELAY, ADP1050_TOFF_DELAY_0MS, 1);
	if (ret)
		return ret;

	ret = adp1050_write(desc, ADP1050_OPERATION, ADP1050_OPERATION_SOFT_OFF
			    | ADP1050_MARGIN_OFF, 1);
	if (ret)
		return ret;

	ret = no_os_gpio_set_value(desc->pg_alt_desc, NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	ret = adp1050_send_command(desc, ADP1050_OPERATION);
	if (ret)
		return ret;

	no_os_gpio_remove(desc->pg_alt_desc);
	no_os_i2c_remove(desc->i2c_desc);
	no_os_free(desc);

	return 0;
}
