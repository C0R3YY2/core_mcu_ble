################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/jv_bt+packet_lib/crc.c \
../Core/Inc/jv_bt+packet_lib/jv_bt+bsc.c \
../Core/Inc/jv_bt+packet_lib/jv_bt+packet.c 

OBJS += \
./Core/Inc/jv_bt+packet_lib/crc.o \
./Core/Inc/jv_bt+packet_lib/jv_bt+bsc.o \
./Core/Inc/jv_bt+packet_lib/jv_bt+packet.o 

C_DEPS += \
./Core/Inc/jv_bt+packet_lib/crc.d \
./Core/Inc/jv_bt+packet_lib/jv_bt+bsc.d \
./Core/Inc/jv_bt+packet_lib/jv_bt+packet.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/jv_bt+packet_lib/%.o Core/Inc/jv_bt+packet_lib/%.su Core/Inc/jv_bt+packet_lib/%.cyclo: ../Core/Inc/jv_bt+packet_lib/%.c Core/Inc/jv_bt+packet_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L432xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-jv_bt-2b-packet_lib

clean-Core-2f-Inc-2f-jv_bt-2b-packet_lib:
	-$(RM) ./Core/Inc/jv_bt+packet_lib/crc.cyclo ./Core/Inc/jv_bt+packet_lib/crc.d ./Core/Inc/jv_bt+packet_lib/crc.o ./Core/Inc/jv_bt+packet_lib/crc.su ./Core/Inc/jv_bt+packet_lib/jv_bt+bsc.cyclo ./Core/Inc/jv_bt+packet_lib/jv_bt+bsc.d ./Core/Inc/jv_bt+packet_lib/jv_bt+bsc.o ./Core/Inc/jv_bt+packet_lib/jv_bt+bsc.su ./Core/Inc/jv_bt+packet_lib/jv_bt+packet.cyclo ./Core/Inc/jv_bt+packet_lib/jv_bt+packet.d ./Core/Inc/jv_bt+packet_lib/jv_bt+packet.o ./Core/Inc/jv_bt+packet_lib/jv_bt+packet.su

.PHONY: clean-Core-2f-Inc-2f-jv_bt-2b-packet_lib

