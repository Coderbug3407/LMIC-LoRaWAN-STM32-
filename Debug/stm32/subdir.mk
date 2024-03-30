################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../stm32/debug.c \
../stm32/eeprom.c \
../stm32/hal.c \
../stm32/hw.c 

OBJS += \
./stm32/debug.o \
./stm32/eeprom.o \
./stm32/hal.o \
./stm32/hw.o 

C_DEPS += \
./stm32/debug.d \
./stm32/eeprom.d \
./stm32/hal.d \
./stm32/hw.d 


# Each subdirectory must supply rules for building sources it contributes
stm32/%.o stm32/%.su stm32/%.cyclo: ../stm32/%.c stm32/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-stm32

clean-stm32:
	-$(RM) ./stm32/debug.cyclo ./stm32/debug.d ./stm32/debug.o ./stm32/debug.su ./stm32/eeprom.cyclo ./stm32/eeprom.d ./stm32/eeprom.o ./stm32/eeprom.su ./stm32/hal.cyclo ./stm32/hal.d ./stm32/hal.o ./stm32/hal.su ./stm32/hw.cyclo ./stm32/hw.d ./stm32/hw.o ./stm32/hw.su

.PHONY: clean-stm32

