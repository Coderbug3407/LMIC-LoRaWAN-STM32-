################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lmic/lmic.c \
../Core/lmic/oslmic.c \
../Core/lmic/radio.c 

OBJS += \
./Core/lmic/lmic.o \
./Core/lmic/oslmic.o \
./Core/lmic/radio.o 

C_DEPS += \
./Core/lmic/lmic.d \
./Core/lmic/oslmic.d \
./Core/lmic/radio.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lmic/%.o Core/lmic/%.su Core/lmic/%.cyclo: ../Core/lmic/%.c Core/lmic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lmic

clean-Core-2f-lmic:
	-$(RM) ./Core/lmic/lmic.cyclo ./Core/lmic/lmic.d ./Core/lmic/lmic.o ./Core/lmic/lmic.su ./Core/lmic/oslmic.cyclo ./Core/lmic/oslmic.d ./Core/lmic/oslmic.o ./Core/lmic/oslmic.su ./Core/lmic/radio.cyclo ./Core/lmic/radio.d ./Core/lmic/radio.o ./Core/lmic/radio.su

.PHONY: clean-Core-2f-lmic

