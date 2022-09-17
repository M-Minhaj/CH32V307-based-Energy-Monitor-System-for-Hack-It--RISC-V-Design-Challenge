################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v30x_it.c \
../User/em.c \
../User/main.c \
../User/moving_average.c \
../User/system_ch32v30x.c 

OBJS += \
./User/ch32v30x_it.o \
./User/em.o \
./User/main.o \
./User/moving_average.o \
./User/system_ch32v30x.o 

C_DEPS += \
./User/ch32v30x_it.d \
./User/em.d \
./User/main.d \
./User/moving_average.d \
./User/system_ch32v30x.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU RISC-V Cross C Compiler'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -I"D:\RISC_V_RT_THREAD\ch32v307-main\EVT\EXAM\SRC\Debug" -I"D:\RISC_V_RT_THREAD\EM\EM\Core" -I"D:\RISC_V_RT_THREAD\EM\EM\User" -I"D:\RISC_V_RT_THREAD\EM\EM\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


