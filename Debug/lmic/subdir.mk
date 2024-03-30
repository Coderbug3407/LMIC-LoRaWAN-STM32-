################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lmic/lmic.c \
../lmic/oslmic.c \
../lmic/radio.c 

OBJS += \
./lmic/lmic.o \
./lmic/oslmic.o \
./lmic/radio.o 

C_DEPS += \
./lmic/lmic.d \
./lmic/oslmic.d \
./lmic/radio.d 


# Each subdirectory must supply rules for building sources it contributes
lmic/%.o lmic/%.su lmic/%.cyclo: ../lmic/%.c lmic/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lmic

clean-lmic:
	-$(RM) ./lmic/lmic.cyclo ./lmic/lmic.d ./lmic/lmic.o ./lmic/lmic.su ./lmic/oslmic.cyclo ./lmic/oslmic.d ./lmic/oslmic.o ./lmic/oslmic.su ./lmic/radio.cyclo ./lmic/radio.d ./lmic/radio.o ./lmic/radio.su

.PHONY: clean-lmic

