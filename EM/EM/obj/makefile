################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/u8g2/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
EM.hex \

SECONDARY_LIST += \
EM.lst \

SECONDARY_SIZE += \
EM.siz \


# All Target
all: EM.elf secondary-outputs

# Tool invocations
EM.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU RISC-V Cross C Linker'
	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wunused -Wuninitialized  -g -T "D:\RISC_V_RT_THREAD\EM\EM\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -L/usr/lib -Wl,-Map,"EM.map" -Xlinker --cref -Xlinker --print-map --specs=nano.specs -u _printf_float -u _scanf_float --specs=nosys.specs -v -o "EM.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

EM.hex: EM.elf
	@echo 'Invoking: GNU RISC-V Cross Create Flash Image'
	riscv-none-embed-objcopy -O ihex "EM.elf"  "EM.hex"
	@echo 'Finished building: $@'
	@echo ' '

EM.lst: EM.elf
	@echo 'Invoking: GNU RISC-V Cross Create Listing'
	riscv-none-embed-objdump --all-headers --demangle --disassemble "EM.elf" > "EM.lst"
	@echo 'Finished building: $@'
	@echo ' '

EM.siz: EM.elf
	@echo 'Invoking: GNU RISC-V Cross Print Size'
	riscv-none-embed-size --format=berkeley "EM.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_UPPER_DEPS)$(C_DEPS) EM.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
