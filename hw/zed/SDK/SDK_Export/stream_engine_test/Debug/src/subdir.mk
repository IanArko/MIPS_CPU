################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dma.c \
../src/platform.c \
../src/stream_engine_test.c 

LD_SRCS += \
../src/lscript.ld 

OBJS += \
./src/dma.o \
./src/platform.o \
./src/stream_engine_test.o 

C_DEPS += \
./src/dma.d \
./src/platform.d \
./src/stream_engine_test.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM gcc compiler'
	arm-xilinx-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 --std=gnu99 -I../../standalone_bsp_0/ps7_cortexa9_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


