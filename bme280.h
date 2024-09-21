#ifndef BME280_H_
#define BME280_H_

#include "bme280_settings.h"

#define __weak   __attribute__((weak))

/**
 * @brief Function-like macro converting number from float to int32
 * @param[in] FLOAT_NUM Float number
 * @return Converted to int32 number
 */
#define BME280_ROUND_FLOAT_TO_INT(FLOAT_NUM) (int32_t)((FLOAT_NUM / (float)(int32_t)FLOAT_NUM) > 1 ? ((float)(int32_t)FLOAT_NUM + 1.) : (float)(int32_t)FLOAT_NUM)

/**
 * @brief BME280 I2C Device address definition. SDO is set in the settings h-file
 */
#if SDO == 0
#define BME280_DEV_ADDR 		0x76
#else
#define BME280_DEV_ADDR 		0x77
#endif

/**
 * @brief BME280 Register addresses definition
 */
#define BME280_REG_CALIB00		0x88
#define BME280_REG_ID 			0xD0
#define BME280_REG_RESET		0xE0
#define BME280_REG_CALIB26		0xE1
#define BME280_REG_CTRL_HUM		0xF2
#define BME280_REG_STATUS		0xF3
#define BME280_REG_CTRL_MEAS	0xF4
#define BME280_REG_CONFIG		0xF5
#define BME280_REG_PRESS_MSB	0xF7
#define BME280_REG_PRESS_LSB	0xF8
#define BME280_REG_PRESS_XLSB	0xF9
#define BME280_REG_TEMP_MSB		0xFA
#define BME280_REG_TEMP_LSB		0xFB
#define BME280_REG_TEMP_XLSB	0xFC
#define BME280_REG_HUM_MSB		0xFD
#define BME280_REG_HUM_LSB		0xFE

/**
 * @brief
 */
#define BME280_DATA_LEN_FROM_CALIB00	26
#define BME280_DATA_LEN_FROM_CALIB26	16
#define BME280_MEASURMENTS_DATA_LEN		8

/**
 * @brief BME280 Register reset values
 *
 */
#define BME280_CHIP_ID 				0x60
#define BME280_REG_RESET_STATE 		0x00
#define BME280_MSB_REG_RESET_STATE 	0x80

/**
 * @brief BME280 Oversampling values definition
 */
#define BME280_MEAS_SKIP		0x00
#define BME280_X1_OVERSAMPLING	0x01
#define BME280_X2_OVERSAMPLING	0x02
#define BME280_X4_OVERSAMPLING	0x03
#define BME280_X8_OVERSAMPLING	0x04
#define BME280_X16_OVERSAMPLING	0x05

/**
 * @brief BME280 Command values definition
 */
#define BME280_SOFT_RESET		0xB6
#define BME280_SLEEP_MODE		0x00
#define BME280_FORCED_MODE1		0x01
#define BME280_FORCED_MODE2		0x02
#define BME280_NORMAL_MODE		0x03

/**
 * @brief BME280 Inactive duration time values definition
 */
#define BME280_STANDBY_1_MS		0x00
#define BME280_STANDBY_63_MS	0x01
#define BME280_STANDBY_125_MS	0x02
#define BME280_STANDBY_250_MS	0x03
#define BME280_STANDBY_500_MS	0x04
#define BME280_STANDBY_1000_MS	0x05
#define BME280_STANDBY_10_MS	0x06
#define BME280_STANDBY_20_MS	0x07

/**
 * @brief BME280 Filter coefficient values definition
 */
#define BME280_FILTER_COEFF_OFF	0x00
#define BME280_FILTER_COEFF_X2	0x01
#define BME280_FILTER_COEFF_X4	0x02
#define BME280_FILTER_COEFF_X8	0x03
#define BME280_FILTER_COEFF_X16	0x04

/**
 * @brief	BME280 Status register bits
 */
#define BME280_STATUS_COPYING		0x01
#define BME280_STATUS_CONVERSION	0x04

/**
 * @brief BME280 Current consumption parameters definition
 */
#define BME280_SLEEP_CURRENT_TYP		0.1		///< Sleep current typ value
#define BME280_SLEEP_CURRENT_MAX		0.3		///< Sleep current max value
#define BME280_STANDBY_CURRENT_TYP		0.2		///< Standby current typ value
#define BME280_STANDBY_CURRENT_MAX		0.5		///< Standby current max value
#define BME280_HUM_MEAS_CURRENT			340		///< Current during humidity measurement
#define BME280_PRESS_MEAS_CURRENT		714		///< Current during pressure measurement
#define BME280_TEMP_MEAS_CURRENT		350		///< Current during temperature measurement

/**
 * @brief BME280 Interface selection
 */
typedef enum BME280_interface {
	I2C=0, SPI
} BME280_interface_t;

/**
 * @brief BME280 Status enum
 */
typedef enum BME280_status {
	BME280_OK=0, BME280_ERROR
} BME280_status_t;

typedef enum BME280_oversampling {
	MEAS_SKIP=0, X1_OVERSAMP, X2_OVERSAMP, X4_OVERSAMP, X8_OVERSAMP, X16_OVERSAMP
} BME280_oversampling_t;

typedef enum BME280_filterCoeff {
	FILTER_OFF=0, FILTER_X2, FILTER_X4, FILTER_X8, FILTER_X16
} BME280_filterCoeff_t;

typedef enum BME280_standbyTime {
	STANDBY_1MS=0, STANDBY_63MS, STANDBY_125MS, STANDBY_250MS, STANDBY_500MS, STANDBY_1000MS, STANDBY_10MS, STANDBY_20MS
} BME280_standbyTime_t;

typedef enum BME280_mode {
	SLEEP_MODE=0, FORCED_MODE=1, NORMAL_MODE=3
} BME280_mode_t;

/**
 * @brief Contains calculated parameters containing information about working time periods and frequencies
 */
typedef struct BME280_calcInfoData {
	float measure_time;				///< Measurement time
	float standby_time;				///< Inactive time in normal mode
	float max_ODR;					///< Maximum output data rate (Hz)
	uint8_t IIR_response_samples;	///< Number of samples to reach 75% of step response with initialized IIR filter
	float IIR_response_time;		///< Response time using IIR filter (to reach 75% of a step response)
	float current_consumption;		///< Current consumption
} BME280_calcInfoData_t;

/**
 * @brief Struct contains measurement settings and information about data rate and current consumption.
 */
typedef struct BME280_measureConfig {
	BME280_oversampling_t temp_oversamp;
	BME280_oversampling_t press_oversamp;
	BME280_oversampling_t hum_oversamp;
	BME280_filterCoeff_t filter_coeff;
	BME280_standbyTime_t standby_time;
	BME280_mode_t mode;				/**< Mode (optional parameter if you pass the struct to functions of only one mode:
	 	 	 	 	 	 	 	< BME280_once_measurement(), BME280_normal_mode_enable()) */
	BME280_calcInfoData_t data_flow_info;	///< Calculated parameters containing information about working time periods and frequencies
} BME280_measureConfig_t;

/**
 * @brief Contains values of part of the sensor registers
 */
typedef struct BME280_regDataHandler {
	uint8_t chip_id;	///< [7:0] - chip id = 0x60
	uint8_t ctrl_hum;	///< [2:0] - humidity oversampling
	uint8_t status;		///< [3] - conversion is running (set); [0] - NVM data are being copying at reset and before every conversion (set)
	uint8_t ctrl_meas;	///< [7:5] - temperature oversampling; [4:2] - pressure oversampling; [1:0] - mode
	uint8_t config;		///< [7:5] - inactive duration time in normal mode; [4:2] - IIR filter coefficient; [0] - set 3-wire SPI interface
	uint8_t press_msb;	///< MSB part of the raw pressure measurement output data
	uint8_t press_lsb;	///< LSB part of the raw pressure measurement output data
	uint8_t press_xlsb;	///< XLSB part of the raw pressure measurement output data ([3:0] - filled with zeros)
	uint8_t temp_msb;	///< MSB part of the raw temperature measurement output data
	uint8_t temp_lsb;	///< LSB part of the raw temperature measurement output data
	uint8_t temp_xlsb;	///< XLSB part of the raw temperature measurement output data ([3:0] - filled with zeros)
	uint8_t hum_msb;	///< MSB part of the raw humidity measurement output data
	uint8_t hum_lsb;	///< LSB part of the raw humidity measurement output data
} BME280_regDataHandler_t;

/**
 * @brief Contains registers (calib00..calib41) values to compensate measurement output data
 */
typedef struct BME280_calibData {
	/**
	 * Temperature compensation words (dig_T#)
	 */
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	/**
	 * Pressure compensation words (dig_P#)
	 */
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	/**
	 * Humidity compensation words (dig_H#)
	 */
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int16_t dig_H6;
} BME280_calibData_t;

/**
 * @brief Contains main data about the sensor
 */
typedef struct BME280_handler {
	uint16_t device_addr;	///< Device address
	uint32_t humidity;		///< Compensated humidity measurement value
	int32_t temperature;	///< Compensated temperature measurement value
	uint32_t pressure;		///< Compensated pressure measurement value
	BME280_calibData_t calibration_data;		///< Calibration values for measurement raw data compensation
	BME280_regDataHandler_t registers_data;		///< Sensor registers values

	BME280_interface_t interface_select;		///< Selected communication interface, SPI/I2C
	void *interface_handler;					///< Pointer to interface descriptor/handler/struct

	BME280_measureConfig_t *current_config;		///< Pointer to struct containing current measurement config
} BME280_handler_t;

void BME280_delay(BME280_handler_t *bme_handler, uint32_t delay);
BME280_status_t BME280_readout_data(BME280_handler_t *bme_handler, uint8_t reg_addr, uint16_t size, uint8_t *read_buffer, uint16_t read_data_len);
BME280_status_t BME280_write_data(BME280_handler_t *bme_handler, uint8_t reg_addr, uint16_t size, uint8_t *write_data, uint16_t write_data_len);

BME280_status_t BME280_init(BME280_handler_t *bme_handler, BME280_interface_t interface_select, void *interface_handler);
BME280_status_t BME280_get_calibration_data(BME280_handler_t *bme_handler);
BME280_status_t BME280_soft_reset(BME280_handler_t *bme_handler);
BME280_status_t BME280_enable_sleep_mode(BME280_handler_t *bme_handler);

BME280_status_t BME280_once_measurement(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct);
BME280_status_t BME280_normal_mode_enable(BME280_handler_t *bme_handler, BME280_measureConfig_t *measure_struct);
BME280_status_t BME280_read_comp_parameters(BME280_handler_t *bme_handler);

void BME280_update_data_flow_info(BME280_measureConfig_t *measure_struct);
float BME280_calc_measure_time(BME280_oversampling_t temp_oversamp, BME280_oversampling_t press_oversamp, BME280_oversampling_t hum_oversamp);
float BME280_calc_standby_time(BME280_standbyTime_t reg_data_standby);
float BME280_calc_data_rate(float measure_time, float standby_time);
uint8_t BME280_calc_response_samples(BME280_filterCoeff_t filter_coeff);
float BME280_calc_response_time(uint8_t response_samples, float out_data_rate);
float BME280_calc_current_consumption(BME280_mode_t mode, float out_data_rate, float measure_time, BME280_oversampling_t temp_oversamp, \
									  BME280_oversampling_t press_oversamp, BME280_oversampling_t hum_oversamp);

#endif /* The end of #ifndef BME280_H_ */
