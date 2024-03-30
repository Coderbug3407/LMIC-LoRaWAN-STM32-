################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/stm32/debug.c \
../Core/stm32/eeprom.c \
../Core/stm32/hal.c \
../Core/stm32/hw.c 

OBJS += \
./Core/stm32/debug.o \
./Core/stm32/eeprom.o \
./Core/stm32/hal.o \
./Core/stm32/hw.o 

C_DEPS += \
./Core/stm32/debug.d \
./Core/stm32/eeprom.d \
./Core/stm32/hal.d \
./Core/stm32/hw.d 


# Each subdirectory must supply rules for building sources it contributes
Core/stm32/%.o Core/stm32/%.su Core/stm32/%.cyclo: ../Core/stm32/%.c Core/stm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-stm32

clean-Core-2f-stm32:
	-$(RM) ./Core/stm32/debug.cyclo ./Core/stm32/debug.d ./Core/stm32/debug.o ./Core/stm32/debug.su ./Core/stm32/eeprom.cyclo ./Core/stm32/eeprom.d ./Core/stm32/eeprom.o ./Core/stm32/eeprom.su ./Core/stm32/hal.cyclo ./Core/stm32/hal.d ./Core/stm32/hal.o ./Core/stm32/hal.su ./Core/stm32/hw.cyclo ./Core/stm32/hw.d ./Core/stm32/hw.o ./Core/stm32/hw.su

.PHONY: clean-Core-2f-stm32

