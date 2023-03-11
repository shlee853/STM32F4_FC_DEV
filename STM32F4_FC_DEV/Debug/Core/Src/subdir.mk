################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GPS_Receiver.c \
../Core/Src/ICM20602.c \
../Core/Src/Quaternion.c \
../Core/Src/SBUS_Receiver.c \
../Core/Src/W25Qxx_Flash.c \
../Core/Src/adc.c \
../Core/Src/circular_queue.c \
../Core/Src/cli.c \
../Core/Src/cli_command.c \
../Core/Src/cli_uart.c \
../Core/Src/common.c \
../Core/Src/dma.c \
../Core/Src/flash_if.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/menu.c \
../Core/Src/spi.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/ymodem.c 

OBJS += \
./Core/Src/GPS_Receiver.o \
./Core/Src/ICM20602.o \
./Core/Src/Quaternion.o \
./Core/Src/SBUS_Receiver.o \
./Core/Src/W25Qxx_Flash.o \
./Core/Src/adc.o \
./Core/Src/circular_queue.o \
./Core/Src/cli.o \
./Core/Src/cli_command.o \
./Core/Src/cli_uart.o \
./Core/Src/common.o \
./Core/Src/dma.o \
./Core/Src/flash_if.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/menu.o \
./Core/Src/spi.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/ymodem.o 

C_DEPS += \
./Core/Src/GPS_Receiver.d \
./Core/Src/ICM20602.d \
./Core/Src/Quaternion.d \
./Core/Src/SBUS_Receiver.d \
./Core/Src/W25Qxx_Flash.d \
./Core/Src/adc.d \
./Core/Src/circular_queue.d \
./Core/Src/cli.d \
./Core/Src/cli_command.d \
./Core/Src/cli_uart.d \
./Core/Src/common.d \
./Core/Src/dma.d \
./Core/Src/flash_if.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/menu.d \
./Core/Src/spi.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/ymodem.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F405xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/GPS_Receiver.d ./Core/Src/GPS_Receiver.o ./Core/Src/GPS_Receiver.su ./Core/Src/ICM20602.d ./Core/Src/ICM20602.o ./Core/Src/ICM20602.su ./Core/Src/Quaternion.d ./Core/Src/Quaternion.o ./Core/Src/Quaternion.su ./Core/Src/SBUS_Receiver.d ./Core/Src/SBUS_Receiver.o ./Core/Src/SBUS_Receiver.su ./Core/Src/W25Qxx_Flash.d ./Core/Src/W25Qxx_Flash.o ./Core/Src/W25Qxx_Flash.su ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/circular_queue.d ./Core/Src/circular_queue.o ./Core/Src/circular_queue.su ./Core/Src/cli.d ./Core/Src/cli.o ./Core/Src/cli.su ./Core/Src/cli_command.d ./Core/Src/cli_command.o ./Core/Src/cli_command.su ./Core/Src/cli_uart.d ./Core/Src/cli_uart.o ./Core/Src/cli_uart.su ./Core/Src/common.d ./Core/Src/common.o ./Core/Src/common.su ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/flash_if.d ./Core/Src/flash_if.o ./Core/Src/flash_if.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/menu.d ./Core/Src/menu.o ./Core/Src/menu.su ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/ymodem.d ./Core/Src/ymodem.o ./Core/Src/ymodem.su

.PHONY: clean-Core-2f-Src

