#ifndef BME280_SETTINGS_H_
#define BME280_SETTINGS_H_

/**
 * @brief SDO pin config. SDO == 0 - SDO is connected to GND; SDO == 1 - SDO is connected to Vddio
 */
#define SDO 0

/**
 * @brief Measurement parameters calculation mode. If define as 0 measurement parameters will be calculated as double type.
 * 		  It gives the best accuracy but is only recommended for PC applications.
 */
#define ENABLE_DOUBLE_PRECISION		0

#define PRESSURE_32BIT_CALC			0

/**
 * @brief SPI mode setting. If SPI_3WIRE == 1 the 3-wire mode is selected.
 */
#define SPI_3WIRE	0

/**
 * @brief BME280 Settings to soft reset. Set the number of attempts to check your sensor is reseted and delay between attempts.
 */
#define BME280_TRY_ATTEMPTS_TO_CHECK_REG	5	///< Number of attempts to check sensor is reseted
#define BME280_WAIT_REG_UPDATE_DELAY		3	///< Delay between attempts to check sensor is reseted (ms)

#define CALCULATE_VALUES_MAX		1

#define UPDATE_CONSUMPTION_INFO 	1

#endif /* The end of #ifndef BME280_SETTINGS_H_ */
