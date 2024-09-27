# BME280 API Library

## Description
 This library is an API for the BME280 humidity, temperature and pressure sensor. The library can be used for: measurements in both single (forced) and continuous (normal) modes for a user-defined configuration; calculation of current consumption, time and frequency parameters of measurements; soft reset of the sensor and swithing to sleep mode; independent work with registers. The user has the ability to independently configure the interface for communicating with the sensor, select methods for compensating for raw measurements and formulas for calculating some parameters. Since the functions of a low layer of communication with the sensor are defined by the user, the library is universal and can be used on various microcontrollers. If your project is written for STM32, you can use the code from the examples in the *Examples* folder.

### Library structure:
*bme280.c* - mian c-file containing function definitions.  
*bme280.h* - main header file containing definitions of structures, enum types, macros  
*bme280_settings.h* - file to customize the library for your project.  
*Examples* - folder with example projects using this library. Includes:  
> STM32_Normal - example of using the sensor in normal mode. For I2C communication, HAL functions are used in blocking mode. 

You can read more about how the library works in the files.

## How to use
### First steps:
- Add library files to your project (*bme280.c*, *bme280.h*, *bme_settings.h*).
- Write your own function bodies in the bme280.c file for next functions: *BME280_delay()*, *BME280_readout_data()*, *BME280_write_data()*. Or use ready-made functions for STM32 from examples.
- Set settings in the bme280_settings.h file.  
- Create *BME280_handler_t* structure and initialize it using *BME280_init()*.
- Use the funtionality you want. You can read all the information about it in the files.

### How to take measurements:
- Create *BME280_measureConfig_t* structure and fill it by necessary properties.
    > *BME280_measureConfig_t measure_cfg = {};  
    measure_cfg.filter_coeff = FILTER_X8;  
    measure_cfg.hum_oversamp = X1_OVERSAMP;  
    measure_cfg.press_oversamp = X4_OVERSAMP;  
    measure_cfg.temp_oversamp = X1_OVERSAMP;*   
- Forced mode: use *BME280_once_measurement()* function (using *BME280_measureConfig_t* structure) to measure selected parameters, read and compensate it.  
Normal mode: use *BME280_normal_mode_enable()* function (using *BME280_measureConfig_t* structure) to configure your sensor by selected parameters and enable normal mode. Then wait for the correct measurements to be established and use *BME280_read_comp_parameters()* to read and compensate measurement raw data.  
- Then you can check the measurements data in the corresponding structure inside the *BME280_handler_t*.  
- The *BME280_once_measurement()* and *BME280_normal_mode_enable()* functions automatically calculate the measurement time, inactive time, maximum output frequency, waiting time for correct measurements to be established, and write this data to the corresponding field of received *BME280_measureConfig_t* structure. You can use this data to set the interval between your measurements.  

### How to do precalculations for your measurement configuration
- Create *BME280_measureConfig_t* structure and fill it by necessary properties (including operating mode).
    > *BME280_measureConfig_t measure_cfg = {};  
    measure_cfg.mode = NORMAL_MODE;  
    measure_cfg.filter_coeff = FILTER_X8;  
    measure_cfg.hum_oversamp = X1_OVERSAMP;  
    measure_cfg.press_oversamp = X4_OVERSAMP;  
    measure_cfg.temp_oversamp = X1_OVERSAMP;*  
- Use *BME280_update_data_flow_info()* function by passing the *BME280_measureConfig_t* structure to it.
- Check the corresponding field of this structure.
