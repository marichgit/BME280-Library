################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/bme280/bme280.c 

OBJS += \
./Core/bme280/bme280.o 

C_DEPS += \
./Core/bme280/bme280.d 


# Each subdirectory must supply rules for building sources it contributes
Core/bme280/%.o Core/bme280/%.su Core/bme280/%.cyclo: ../Core/bme280/%.c Core/bme280/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/stm32_projects/BME280-Library/STM32 Examples/STM32_Normal/Core/bme280" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-bme280

clean-Core-2f-bme280:
	-$(RM) ./Core/bme280/bme280.cyclo ./Core/bme280/bme280.d ./Core/bme280/bme280.o ./Core/bme280/bme280.su

.PHONY: clean-Core-2f-bme280

