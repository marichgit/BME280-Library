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

	if(BME280_write_data(bme_handler, BME280_REG_RESET, 1, &write_reset, 1) != BME280_OK)
		return BME280_ERROR;

	uint8_t status_reg;
	do {
		result = BME280_readout_data(bme_handler, BME280_REG_STATUS, 1, &status_reg, 1);
		BME280_delay(bme_handler, BME280_WAIT_REG_UPDATE_DELAY);
		try_temp--;
	}
	while((status_reg & BME280_STATUS_COPYING) && result == BME280_OK && try_temp != 0);

	if(result == BME280_ERROR || (status_reg & BME280_STATUS_COPYING))
		return BME280_ERROR;

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

int32_t t_fine;
int32_t BME280_compensate_temp_int32(BME280_calibData_t *calib_data, int32_t uncomp_temp) {
	int32_t var1, var2, temp;
	var1 = ((((uncomp_temp>>3) - ((int32_t)calib_data->dig_T1<<1))) * ((int32_t)calib_data->dig_T2)) >> 11;
	var2 = (((((uncomp_temp>>4) - ((int32_t)calib_data->dig_T1)) * (uncomp_temp>>4) - ((int32_t)calib_data->dig_T1))) >> 12) * \
		   ((int32_t)calib_data->dig_T3) >> 14;
	t_fine = var1 + var2;
	temp = (t_fine * 5 + 128) >> 8;

	int32_t temperature_min = BME280_TEMPERATURE_MIN * 100;
	int32_t temperature_max = BME280_TEMPERATURE_MAX * 100;
	if(temp < (temperature_min))
		temp = temperature_min;
	else if(temp > temperature_max)
		temp = temperature_max;

	return temp;
}

uint32_t BME280_compensate_press_int64(BME280_calibData_t *calib_data, int32_t uncomp_press) {
	int64_t var1, var2, press;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calib_data->dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data->dig_P5)<<17);
	var2 = var2 + (((int64_t)calib_data->dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calib_data->dig_P3)>>8) + ((var1 * (int64_t)calib_data->dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t)calib_data->dig_P1)>>33;
	if(var1 == 0)
		return 0;
	press = 1048576 - uncomp_press;
	press = (((press<<31) - var2) * 3125) / var1;
	var1 = ((int64_t)calib_data->dig_P9) * (press>>13) * (press>>13) >> 25;
	var2 = (((int64_t)calib_data->dig_P8) * press) >> 19;
	press = ((press + var1 + var2) >> 8) + (((int64_t)calib_data->dig_P7) << 4);

	return (uint32_t)press;
}

uint32_t BME280_compensate_press_int32(BME280_calibData_t *calib_data, int32_t uncomp_press) {
	int32_t var1, var2;
	uint32_t press;
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11) * (int32_t)calib_data->dig_P6;
	var2 = var2 + ((var1 * ((int32_t)calib_data->dig_P5))<<1);
	var2 = (var2>>2) + (((int32_t)calib_data->dig_P4)<<16);
	var1 = (((calib_data->dig_P3 * (((var1>>2) * (var1>>2)) >> 13)) >> 3) + ((((int32_t)calib_data->dig_P2) * var1) >> 1)) >> 18;
	var1 = (((32768 + var1)) * ((int32_t)calib_data->dig_P1)) >> 15;
	if(var1 == 0)
		return 0;
	press = (((uint32_t)(((int32_t)1048576) - uncomp_press) - (var2 >> 12))) * 3125;
	if(press < 0x80000000)
		press = (press << 1) / ((uint32_t)var1);
	else
		press = (press / (uint32_t)var1) * 2;
	var1 = (((int32_t)calib_data->dig_P9) * ((int32_t)(((press>>3) * (press>>3)) >> 13))) >> 12;
	var2 = (((int32_t)(press>>2)) * ((int32_t)calib_data->dig_P8)) >> 13;
	press = (uint32_t)((int32_t)press + ((var1 + var2 + calib_data->dig_P7) >> 4));

	return press;
}

uint32_t BME280_compensate_hum_int32(BME280_calibData_t *calib_data, int32_t uncomp_hum) {
	int32_t v_x1_u32r;

	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((uncomp_hum << 14) - (((int32_t)calib_data->dig_H4) << 20) - (((int32_t)calib_data->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) \
				* (((((((v_x1_u32r * ((int32_t)calib_data->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_data->dig_H3)) >> 11) + ((int32_t)32768))) \
				>> 10) + ((int32_t)2097152)) * ((int32_t)calib_data->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

	return (uint32_t)(v_x1_u32r >> 12);
}

double BME280_compensate_temp_double(BME280_calibData_t *calib_data, int32_t uncomp_temp) {
	double var1;
	double var2;
	double temperature;
	double temperature_min = (double)BME280_TEMPERATURE_MIN;
	double temperature_max = (double)BME280_TEMPERATURE_MAX;

	var1 = (((double)uncomp_temp) / 16384.0 - ((double)calib_data->dig_T1) / 1024.0);
	var1 = var1 * ((double)calib_data->dig_T2);
	var2 = (((double)uncomp_temp) / 131072.0 - ((double)calib_data->dig_T1) / 8192.0);
	var2 = (var2 * var2) * ((double)calib_data->dig_T3);
	t_fine = (int32_t)(var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min)
	{
		temperature = temperature_min;
	}
	else if (temperature > temperature_max)
	{
		temperature = temperature_max;
	}

	return temperature;
}

double BME280_compensate_press_double(BME280_calibData_t *calib_data, int32_t uncomp_press) {
	double var1, var2, var3, pressure;
	double pressure_min = (double)BME280_PRESSURE_MIN;
	double pressure_max = (double)BME280_PRESSURE_MAX;

	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib_data->dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)calib_data->dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib_data->dig_P4) * 65536.0);
	var3 = ((double)calib_data->dig_P3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)calib_data->dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)calib_data->dig_P1);

	if (var1 == 0)
		return 0;
	pressure = 1048576.0 - (double)uncomp_press;
	pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)calib_data->dig_P9) * pressure * pressure / 2147483648.0;
	var2 = pressure * ((double)calib_data->dig_P8) / 32768.0;
	pressure = pressure + (var1 + var2 + ((double)calib_data->dig_P7)) / 16.0;

	if (pressure < pressure_min)
	{
		pressure = pressure_min;
	}
	else if (pressure > pressure_max)
	{
		pressure = pressure_max;
	}

	return pressure;
}

double BME280_compensate_hum_double(BME280_calibData_t *calib_data, int32_t uncomp_hum) {
	double humidity;
	double humidity_min = (double)BME280_HUMIDITY_MIN;
	double humidity_max = (double)BME280_HUMIDITY_MAX;
	double var1, var2, var3, var4, var5, var6;

	var1 = ((double)t_fine) - 76800.0;
	var2 = (((double)calib_data->dig_H4) * 64.0 + (((double)calib_data->dig_H5) / 16384.0) * var1);
	var3 = uncomp_hum - var2;
	var4 = ((double)calib_data->dig_H2) / 65536.0;
	var5 = (1.0 + (((double)calib_data->dig_H3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)calib_data->dig_H6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - ((double)calib_data->dig_H1) * var6 / 524288.0);

	if (humidity > humidity_max)
	{
		humidity = humidity_max;
	}
	else if (humidity < humidity_min)
	{
		humidity = humidity_min;
	}

	return humidity;
}

BME280_status_t BME280_read_comp_parameters(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct) {
	uint8_t read_buffer[8];
	if(BME280_readout_data(bme_handler, BME280_REG_PRESS_MSB, BME280_MEASURMENTS_DATA_LEN, read_buffer, (uint16_t)sizeof(read_buffer)) != BME280_OK)
		return BME280_ERROR;

#if ENABLE_DOUBLE_PRECISION == 1
	if(measure_struct->press_oversamp != MEAS_SKIP) {
		bme_handler->uncomp_parameters.uncomp_pressure = (read_buffer[0] << 12) | (read_buffer[1] << 4) | (read_buffer[2] >> 4);
		bme_handler->comp_parameters.pressure = BME280_compensate_press_double(&bme_handler->calibration_data, \
																			   bme_handler->uncomp_parameters.uncomp_pressure);
	}
	if(measure_struct->temp_oversamp != MEAS_SKIP) {
		bme_handler->uncomp_parameters.uncomp_temperature = (read_buffer[3] << 12) | (read_buffer[4] << 4) | (read_buffer[5] >> 4);
		bme_handler->comp_parameters.temperature = BME280_compensate_temp_double(&bme_handler->calibration_data, \
																				 bme_handler->uncomp_parameters.uncomp_temperature);
	}
	if(measure_struct->hum_oversamp != MEAS_SKIP) {
		bme_handler->uncomp_parameters.uncomp_humidity = (read_buffer[6] << 8) | read_buffer[7];
		bme_handler->comp_parameters.humidity = BME280_compensate_hum_double(&bme_handler->calibration_data, bme_handler->uncomp_parameters.uncomp_humidity);
	}
#else
	if(measure_struct->press_oversamp != MEAS_SKIP) {
		bme_handler->uncomp_parameters.uncomp_pressure = (read_buffer[0] << 12) | (read_buffer[1] << 4) | (read_buffer[2] >> 4);
	#if PRESSURE_32BIT_CALC == 0
		bme_handler->comp_parameters.pressure = BME280_compensate_press_int64(&bme_handler->calibration_data, bme_handler->uncomp_parameters.uncomp_pressure);
	#else
		bme_handler->comp_parameters.pressure = BME280_compensate_press_int32(&bme_handler->calibration_data, bme_handler->uncomp_parameters.uncomp_pressure);
	#endif /* End of #if PRESSURE_32BIT_CALC == 1*/
	}
	if(measure_struct->temp_oversamp != MEAS_SKIP) {
		bme_handler->uncomp_parameters.uncomp_temperature = (read_buffer[3] << 12) | (read_buffer[4] << 4) | (read_buffer[5] >> 4);
		bme_handler->comp_parameters.temperature = BME280_compensate_temp_int32(&bme_handler->calibration_data, \
																				bme_handler->uncomp_parameters.uncomp_temperature);
	}
	if(measure_struct->hum_oversamp != MEAS_SKIP) {
		bme_handler->uncomp_parameters.uncomp_humidity = (read_buffer[6] << 8) | read_buffer[7];
		bme_handler->comp_parameters.humidity = BME280_compensate_hum_int32(&bme_handler->calibration_data, bme_handler->uncomp_parameters.uncomp_humidity);
	}
#endif /* End of #if ENABLE_DOUBLE_PRECISION == 1*/

	return BME280_OK;
}

BME280_status_t BME280_normal_mode_enable(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct) {
	measure_struct->mode = NORMAL_MODE;
	bme_handler->current_config = measure_struct;

	uint8_t write_data = (SPI_3WIRE) | measure_struct->filter_coeff << 2 | measure_struct->standby_time << 5;
	if(BME280_write_data(bme_handler, BME280_REG_CONFIG, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;

	write_data = measure_struct->hum_oversamp;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_HUM, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;

	write_data = BME280_NORMAL_MODE | measure_struct->press_oversamp << 2 | measure_struct->temp_oversamp << 5;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_MEAS, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;

	BME280_update_data_flow_info(measure_struct);

	return BME280_OK;
}

BME280_status_t BME280_once_measurement(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct) {
	measure_struct->mode = FORCED_MODE;
	bme_handler->current_config = measure_struct;

	uint8_t write_data = (SPI_3WIRE) | measure_struct->filter_coeff << 2;
	if(BME280_write_data(bme_handler, BME280_REG_CONFIG, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;

	write_data = measure_struct->hum_oversamp;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_HUM, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;

	write_data = BME280_FORCED_MODE1 | measure_struct->press_oversamp << 2 | measure_struct->temp_oversamp << 5;
	if(BME280_write_data(bme_handler, BME280_REG_CTRL_MEAS, 1, &write_data, 1) != BME280_OK)
		return BME280_ERROR;

	BME280_update_data_flow_info(measure_struct);
	BME280_delay(bme_handler, (uint32_t)BME280_ROUND_FLOAT_TO_INT(measure_struct->data_flow_info.measure_time));

	if(BME280_read_comp_parameters(bme_handler, measure_struct) != BME280_OK)
		return BME280_ERROR;

	return BME280_OK;
}

void BME280_update_data_flow_info(BME280_measureConfig_t *measure_struct) {
	if(measure_struct->mode == SLEEP_MODE) {
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
		if(measure_struct->mode == NORMAL_MODE)
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

float BME280_calc_measure_time(BME280_oversampling_t temp_oversamp, BME280_oversampling_t press_oversamp, BME280_oversampling_t hum_oversamp) {
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

float BME280_calc_standby_time(BME280_standbyTime_t reg_data_standby) {
	float result;
	switch(reg_data_standby) {
		case STANDBY_1MS:
			result = 0.5;
			break;
		case STANDBY_63MS:
			result = 62.5;
			break;
		case STANDBY_125MS:
			result = 125.;
			break;
		case STANDBY_250MS:
			result = 250.;
			break;
		case STANDBY_500MS:
			result = 500.;
			break;
		case STANDBY_1000MS:
			result = 1000.;
			break;
		case STANDBY_10MS:
			result = 10.;
			break;
		case STANDBY_20MS:
			result = 20.;
	}

	return result;
}

float BME280_calc_data_rate(float measure_time, float standby_time) {
	float out_data_rate = 1000 / (measure_time + standby_time);

	return out_data_rate;
}

uint8_t BME280_calc_response_samples(BME280_filterCoeff_t filter_coeff) {
	uint8_t response_samples;
	switch(filter_coeff) {
		case FILTER_OFF:
			response_samples = 1;
			break;
		case FILTER_X2:
			response_samples = 3;
			break;
		case FILTER_X4:
			response_samples = 8;
			break;
		case FILTER_X8:
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

float BME280_calc_current_consumption(BME280_mode_t mode, float out_data_rate, float measure_time, BME280_oversampling_t temp_oversamp, \
		  	  	  	  	  	  	  	  BME280_oversampling_t press_oversamp, BME280_oversampling_t hum_oversamp) {
	float current_consumption;

#if CALCULATE_VALUES_MAX == 0
	if(mode == NORMAL_MODE)
		current_consumption = BME280_STANDBY_CURRENT_TYP * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
	else if(mode == FORCED_MODE)
		current_consumption = BME280_SLEEP_CURRENT_TYP * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
#else
	if(mode == NORMAL_MODE)
		current_consumption = BME280_STANDBY_CURRENT_MAX * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
	else if(mode == FORCED_MODE)
		current_consumption = BME280_SLEEP_CURRENT_MAX * (1. - measure_time * out_data_rate) + out_data_rate / 1000. * (205. + BME280_TEMP_MEAS_CURRENT * \
							  2. * temp_oversamp + BME280_PRESS_MEAS_CURRENT * (2. * (float)press_oversamp + 0.5) * (press_oversamp != 0) + \
							  BME280_HUM_MEAS_CURRENT * (2. * (float)hum_oversamp + 0.5) * (hum_oversamp != 0));
#endif
	return current_consumption;
}
