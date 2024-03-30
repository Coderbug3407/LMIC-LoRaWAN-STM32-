################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/aes/lmic.c \
../Core/aes/other.c 

OBJS += \
./Core/aes/lmic.o \
./Core/aes/other.o 

C_DEPS += \
./Core/aes/lmic.d \
./Core/aes/other.d 


# Each subdirectory must supply rules for building sources it contributes
Core/aes/%.o Core/aes/%.su Core/aes/%.cyclo: ../Core/aes/%.c Core/aes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-aes

clean-Core-2f-aes:
	-$(RM) ./Core/aes/lmic.cyclo ./Core/aes/lmic.d ./Core/aes/lmic.o ./Core/aes/lmic.su ./Core/aes/other.cyclo ./Core/aes/other.d ./Core/aes/other.o ./Core/aes/other.su

.PHONY: clean-Core-2f-aes

