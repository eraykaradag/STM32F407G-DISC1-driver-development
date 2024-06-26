################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f4xx_gpio.c \
../drivers/Src/stm32f4xx_i2c.c \
../drivers/Src/stm32f4xx_spi.c \
../drivers/Src/stm32f4xx_usart.c 

OBJS += \
./drivers/Src/stm32f4xx_gpio.o \
./drivers/Src/stm32f4xx_i2c.o \
./drivers/Src/stm32f4xx_spi.o \
./drivers/Src/stm32f4xx_usart.o 

C_DEPS += \
./drivers/Src/stm32f4xx_gpio.d \
./drivers/Src/stm32f4xx_i2c.d \
./drivers/Src/stm32f4xx_spi.d \
./drivers/Src/stm32f4xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"C:/Users/PC/Documents/embedded/MCU/stm32f4xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f4xx_gpio.cyclo ./drivers/Src/stm32f4xx_gpio.d ./drivers/Src/stm32f4xx_gpio.o ./drivers/Src/stm32f4xx_gpio.su ./drivers/Src/stm32f4xx_i2c.cyclo ./drivers/Src/stm32f4xx_i2c.d ./drivers/Src/stm32f4xx_i2c.o ./drivers/Src/stm32f4xx_i2c.su ./drivers/Src/stm32f4xx_spi.cyclo ./drivers/Src/stm32f4xx_spi.d ./drivers/Src/stm32f4xx_spi.o ./drivers/Src/stm32f4xx_spi.su ./drivers/Src/stm32f4xx_usart.cyclo ./drivers/Src/stm32f4xx_usart.d ./drivers/Src/stm32f4xx_usart.o ./drivers/Src/stm32f4xx_usart.su

.PHONY: clean-drivers-2f-Src

