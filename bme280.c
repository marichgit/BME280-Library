#include "main.h"
#include "bme280.h"
#include <stdlib.h>

__weak void BME280_delay(BME280_handler_t *bme_handler, uint32_t delay) {
	HAL_Delay(delay);
}

__weak BME280_status_t BME280_readout_data(BME280_handler_t *bme_handler, uint8_t reg_addr, uint16_t size, uint8_t *read_buffer, uint16_t read_data_len) {
	if(size == 0 || size > read_data_len || bme_handler == NULL || read_buffer == NULL)
		return BME280_ERROR;

	if(HAL_I2C_Mem_Read((I2C_HandleTypeDef *)bme_handler->interface_handler, bme_handler->device_addr << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, \
						read_buffer, size, HAL_MAX_DELAY) == HAL_BUSY)
		return BME280_ERROR;

	return BME280_OK;
}

/**
 * @brief This function is used to send data to your sensor. You should rewrite this function if you want to use other interface communication functions.
 *
 * @param[in] bme_handler BME280 Handler structure
 * @param[in] reg_addr Start register address into which the information is written
 * @param[in] size Number of data bytes needs to be sent to the sensor
 * @param[in] write_data Buffer contains data needs to be sent to the sensor
 * @param[in] write_data_len Size of write_data buffer in bytes
 *
 * @return Function result status
 */
__weak BME280_status_t BME280_write_data(BME280_handler_t *bme_handler, uint8_t reg_addr, uint16_t size, uint8_t *write_data, uint16_t write_data_len) {
	if(size == 0 || size > write_data_len || bme_handler == NULL || write_data == NULL)
		return BME280_ERROR;

	uint8_t *write_buffer;
	size_t write_len = (size_t)size*2;

	write_buffer = malloc(write_len);
	if(write_buffer == NULL)
		return BME280_ERROR;
	uint8_t temp_reg = reg_addr;
	for(int i = 0; i < write_len; i++) {
		if(i % 2) {
			write_buffer[i] = temp_reg;
			temp_reg++;
		}
		else
			write_buffer[i] = write_data[i/2];
	}

	BME280_status_t result;
	result = HAL_I2C_Master_Transmit((I2C_HandleTypeDef *)bme_handler->interface_handler, bme_handler->device_addr << 1, write_buffer, \
									write_len, HAL_MAX_DELAY);
	free(write_buffer);

	if(result == BME280_ERROR)
		return BME280_ERROR;

	return BME280_OK;
}

BME280_status_t BME280_init(BME280_handler_t *bme_handler, BME280_interface_t interface_select, void *interface_handler) {
	bme_handler->device_addr = BME280_DEV_ADDR;
	bme_handler->interface_select = interface_select;
	bme_handler->interface_handler = interface_handler;
	uint8_t chip_id;
	if(BME280_readout_data(bme_handler, BME280_REG_ID, 1, &chip_id, 1) != BME280_OK)
		return BME280_ERROR;

	if(chip_id == BME280_CHIP_ID) {
		bme_handler->registers_data.chip_id = chip_id;
		if(BME280_soft_reset(bme_handler) != BME280_OK)
			return BME280_ERROR;

		if(BME280_get_calibration_data(bme_handler) != BME280_OK)
			return BME280_ERROR;
	}
	else
		return BME280_ERROR;


	return BME280_OK;
}

BME280_status_t BME280_soft_reset(BME280_handler_t *bme_handler) {
	uint8_t try_temp = BME280_TRY_ATTEMPTS_TO_CHECK_REG;
	uint8_t write_reset = BME280_SOFT_RESET;
	BME280_status_t result;

	if(BME280_write_data(bme_handler, BME280_REG_RESET, 1, &write_reset, 1) == BME280_ERROR)
		return BME280_ERROR;
	else
		do {
			result = BME280_readout_data(bme_handler, BME280_REG_STATUS, 1, &bme_handler->registers_data.status, 1);
			BME280_delay(bme_handler, BME280_WAIT_REG_UPDATE_DELAY);
			try_temp--;
		}
		while((bme_handler->registers_data.status & BME280_STATUS_COPYING) && result == BME280_OK && try_temp != 0);
	if(result == BME280_ERROR || (bme_handler->registers_data.status & BME280_STATUS_COPYING))
		return BME280_ERROR;

	bme_handler->registers_data.ctrl_hum = BME280_REG_RESET_STATE;
	bme_handler->registers_data.ctrl_meas = BME280_REG_RESET_STATE;
	bme_handler->registers_data.config = BME280_REG_RESET_STATE;
	bme_handler->registers_data.press_msb = BME280_MSB_REG_RESET_STATE;
	bme_handler->registers_data.press_lsb = BME280_REG_RESET_STATE;
	bme_handler->registers_data.press_xlsb = BME280_REG_RESET_STATE;
	bme_handler->registers_data.temp_msb = BME280_MSB_REG_RESET_STATE;
	bme_handler->registers_data.temp_lsb = BME280_REG_RESET_STATE;
	bme_handler->registers_data.temp_xlsb = BME280_REG_RESET_STATE;
	bme_handler->registers_data.hum_msb = BME280_MSB_REG_RESET_STATE;
	bme_handler->registers_data.hum_lsb = BME280_REG_RESET_STATE;
	return BME280_OK;
}

BME280_status_t BME280_get_calibration_data(BME280_handler_t *bme_handler) {
	uint8_t read_buffer[26];

	if(BME280_readout_data(bme_handler, BME280_REG_CALIB00, BME280_DATA_LEN_FROM_CALIB00, read_buffer, (uint16_t)sizeof(read_buffer)) == BME280_ERROR)
		return BME280_ERROR;
	else {
		bme_handler->calibration_data.dig_T1 = (uint16_t)read_buffer[0] << 8 | (uint16_t)read_buffer[1];
		bme_handler->calibration_data.dig_T2 = (uint16_t)read_buffer[2] << 8 | (uint16_t)read_buffer[3];
		bme_handler->calibration_data.dig_T3 = (uint16_t)read_buffer[4] << 8 | (uint16_t)read_buffer[5];
		bme_handler->calibration_data.dig_P1 = (uint16_t)read_buffer[6] << 8 | (uint16_t)read_buffer[7];
		bme_handler->calibration_data.dig_P2 = (uint16_t)read_buffer[8] << 8 | (uint16_t)read_buffer[9];
		bme_handler->calibration_data.dig_P3 = (uint16_t)read_buffer[10] << 8 | (uint16_t)read_buffer[11];
		bme_handler->calibration_data.dig_P4 = (uint16_t)read_buffer[12] << 8 | (uint16_t)read_buffer[13];
		bme_handler->calibration_data.dig_P5 = (uint16_t)read_buffer[14] << 8 | (uint16_t)read_buffer[15];
		bme_handler->calibration_data.dig_P6 = (uint16_t)read_buffer[16] << 8 | (uint16_t)read_buffer[17];
		bme_handler->calibration_data.dig_P7 = (uint16_t)read_buffer[18] << 8 | (uint16_t)read_buffer[19];
		bme_handler->calibration_data.dig_P8 = (uint16_t)read_buffer[20] << 8 | (uint16_t)read_buffer[21];
		bme_handler->calibration_data.dig_P9 = (uint16_t)read_buffer[22] << 8 | (uint16_t)read_buffer[23];
		bme_handler->calibration_data.dig_H1 = (uint8_t)read_buffer[25];
	}
		if(BME280_readout_data(bme_handler, BME280_REG_CALIB26, BME280_DATA_LEN_FROM_CALIB26, read_buffer, (uint16_t)sizeof(read_buffer)) == BME280_ERROR)
			return BME280_ERROR;
		else {
			bme_handler->calibration_data.dig_H2 = (uint16_t)read_buffer[0] << 8 | (uint16_t)read_buffer[1];
			bme_handler->calibration_data.dig_H3 = (uint8_t)read_buffer[2];
			bme_handler->calibration_data.dig_H4 = (uint16_t)read_buffer[3] << 4 | ((uint16_t)read_buffer[4] & 0x0F);
			bme_handler->calibration_data.dig_H5 = (uint16_t)read_buffer[4] >> 4 | (uint16_t)read_buffer[5] << 4;
			bme_handler->calibration_data.dig_H6 = (uint16_t)read_buffer[6];
		}

	return BME280_OK;
}

BME280_status_t BME280_normal_mode_enable(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct) {
	measure_struct->mode = BME280_NORMAL_MODE;
	bme_handler->current_config = measure_struct;

	uint8_t write_data = (bme_handler->registers_data.config & 1) | measure_struct->filter_coeff << 2 | measure_struct->standby_time << 5;
	if(BME280_write_data(bme_handler, BME280_REG_CONFIG, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;
	bme_handler->registers_data.config = write_data;

	write_data = measure_struct->hum_oversamp;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_HUM, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;
	bme_handler->registers_data.ctrl_hum = write_data;

	write_data = BME280_NORMAL_MODE | measure_struct->press_oversamp << 2 | measure_struct->temp_oversamp << 5;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_MEAS, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;
	bme_handler->registers_data.ctrl_meas = write_data;

	BME280_update_data_flow_info(measure_struct);

	return BME280_OK;
}

BME280_status_t BME280_read_comp_parameters(BME280_handler_t *bme_handler) {
	uint8_t read_buffer[8];
	if(BME280_readout_data(bme_handler, BME280_REG_PRESS_MSB, BME280_MEASURMENTS_DATA_LEN, read_buffer, (uint16_t)sizeof(read_buffer)) != BME280_OK)
		return BME280_ERROR;

	bme_handler->registers_data.press_msb = read_buffer[0];
	bme_handler->registers_data.press_lsb = read_buffer[1];
	bme_handler->registers_data.press_xlsb = read_buffer[2];
	bme_handler->registers_data.temp_msb = read_buffer[3];
	bme_handler->registers_data.temp_lsb = read_buffer[4];
	bme_handler->registers_data.temp_xlsb = read_buffer[5];
	bme_handler->registers_data.hum_msb = read_buffer[6];
	bme_handler->registers_data.hum_lsb = read_buffer[7];

	//compensate registers

	return BME280_OK;
}

BME280_status_t BME280_once_measurement(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct) {
	measure_struct->mode = BME280_FORCED_MODE1;
	bme_handler->current_config = measure_struct;

	uint8_t write_data = (bme_handler->registers_data.config & 1) | measure_struct->filter_coeff << 2 | (bme_handler->registers_data.config & 0xE0);
	if(BME280_write_data(bme_handler, BME280_REG_CONFIG, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;
	bme_handler->registers_data.config = write_data;

	write_data = measure_struct->hum_oversamp;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_HUM, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;
	bme_handler->registers_data.ctrl_hum = write_data;

	write_data = BME280_FORCED_MODE1 | measure_struct->press_oversamp << 2 | measure_struct->temp_oversamp << 5;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_MEAS, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;
	bme_handler->registers_data.ctrl_meas = write_data;

	BME280_update_data_flow_info(measure_struct);
	BME280_delay(bme_handler, (uint32_t)BME280_ROUND_FLOAT_TO_INT(measure_struct->data_flow_info.measure_time));

	bme_handler->registers_data.ctrl_meas = bme_handler->registers_data.ctrl_meas & 0xFC;
	if(BME280_read_comp_parameters(bme_handler) != BME280_OK)
		return BME280_ERROR;

	return BME280_OK;
}

void BME280_update_data_flow_info(BME280_measureConfig_t *measure_struct) {
	if(measure_struct->mode == BME280_SLEEP_MODE) {
		measure_struct->data_flow_info.measure_time = 0.;
		measure_struct->data_flow_info.standby_time = 0.;
		measure_struct->data_flow_info.max_ODR = 0.;
		measure_struct->data_flow_info.IIR_response_samples = 0.;
		measure_struct->data_flow_info.IIR_response_time = 0.;
		measure_struct->data_flow_info.current_consumption = 0.;
	}
	else {
		measure_struct->data_flow_info.measure_time = BME280_calc_measure_time(measure_struct->temp_oversamp, measure_struct->press_oversamp, \
																			   measure_struct->hum_oversamp);
		if(measure_struct->mode == BME280_NORMAL_MODE)
			measure_struct->data_flow_info.standby_time = BME280_calc_standby_time(measure_struct->standby_time);
		else
			measure_struct->data_flow_info.standby_time = 0.;
		measure_struct->data_flow_info.max_ODR = BME280_calc_data_rate(measure_struct->data_flow_info.measure_time, \
																	   measure_struct->data_flow_info.standby_time);
		measure_struct->data_flow_info.IIR_response_samples = BME280_calc_response_samples(measure_struct->filter_coeff);
		measure_struct->data_flow_info.IIR_response_time = BME280_calc_response_time(measure_struct->data_flow_info.IIR_response_samples, \
																					 measure_struct->data_flow_info.max_ODR);

#if UPDATE_CONSUMPTION_INFO == 1
	BME280_calc_current_consumption(measure_struct->mode, measure_struct->data_flow_info.max_ODR, measure_struct->data_flow_info.measure_time, \
									measure_struct->temp_oversamp, measure_struct->press_oversamp, measure_struct->hum_oversamp);
#endif
	}
}

float BME280_calc_measure_time(uint8_t temp_oversamp, uint8_t press_oversamp, uint8_t hum_oversamp) {
	float measure_time;
#if CALCULATE_VALUES_MAX == 0
	measure_time = 1. + 2. * (float)temp_oversamp + (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
				  (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0);
#else
	measure_time = 1.25 + 2.3 * (float)temp_oversamp + (2.3 * (float)press_oversamp + 0.575) * (press_oversamp != 0) + \
				  (2.3 * (float)hum_oversamp + 0.575) * (hum_oversamp != 0);
#endif

	return measure_time;
}

float BME280_calc_standby_time(uint8_t reg_data_standby) {
	float result;
	switch(reg_data_standby) {
		case BME280_STANDBY_1_MS:
			result = 0.5;
			break;
		case BME280_STANDBY_63_MS:
			result = 62.5;
			break;
		case BME280_STANDBY_125_MS:
			result = 125.;
			break;
		case BME280_STANDBY_250_MS:
			result = 250.;
			break;
		case BME280_STANDBY_500_MS:
			result = 500.;
			break;
		case BME280_STANDBY_1000_MS:
			result = 1000.;
			break;
		case BME280_STANDBY_10_MS:
			result = 10.;
			break;
		case BME280_STANDBY_20_MS:
			result = 20.;
	}

	return result;
}

float BME280_calc_data_rate(float measure_time, float standby_time) {
	float out_data_rate = 1000 / (measure_time + standby_time);

	return out_data_rate;
}

uint8_t BME280_calc_response_samples(uint8_t filter_coeff) {
	uint8_t response_samples;
	switch(filter_coeff) {
		case BME280_FILTER_COEFF_OFF:
			response_samples = 1;
			break;
		case BME280_FILTER_COEFF_X2:
			response_samples = 3;
			break;
		case BME280_FILTER_COEFF_X4:
			response_samples = 8;
			break;
		case BME280_FILTER_COEFF_X8:
			response_samples = 11;
			break;
		default:
			response_samples = 16;
			break;
	}

	return response_samples;
}

float BME280_calc_response_time(uint8_t response_samples, float out_data_rate) {
	float response_time = 1000 * response_samples / out_data_rate;

	return response_time;
}

float BME280_calc_current_consumption(uint8_t mode, float out_data_rate, float measure_time, uint8_t temp_oversamp, uint8_t press_oversamp, \
									  uint8_t hum_oversamp) {
	float current_consumption;

#if CALCULATE_VALUES_MAX == 0
	if(mode == BME280_NORMAL_MODE)
		current_consumption = BME280_STANDBY_CURRENT_TYP * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
	else if(mode == BME280_FORCED_MODE1)
		current_consumption = BME280_SLEEP_CURRENT_TYP * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
#else
	if(mode == BME280_NORMAL_MODE)
		current_consumption = BME280_STANDBY_CURRENT_MAX * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
	else if(mode == BME280_FORCED_MODE1)
		current_consumption = BME280_SLEEP_CURRENT_MAX * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
#endif
	return current_consumption;
}
