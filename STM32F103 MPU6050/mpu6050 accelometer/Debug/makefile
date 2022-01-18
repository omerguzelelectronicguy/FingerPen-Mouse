################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/HID/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
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
EXECUTABLES += \
mpu6050\ accelometer.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
mpu6050\ accelometer.list \

OBJCOPY_BIN += \
mpu6050\ accelometer.bin \


# All Target
all: mpu6050\ accelometer.elf secondary-outputs

# Tool invocations
mpu6050\ accelometer.elf: $(OBJS) $(USER_OBJS) C:\Users\mrgzl\Documents\GitHub\FingerPen-Mouse\STM32F103\ MPU6050\mpu6050\ accelometer\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "mpu6050 accelometer.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\mrgzl\Documents\GitHub\FingerPen-Mouse\STM32F103 MPU6050\mpu6050 accelometer\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="mpu6050 accelometer.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

mpu6050\ accelometer.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "mpu6050 accelometer.list"
	@echo 'Finished building: $@'
	@echo ' '

mpu6050\ accelometer.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "mpu6050 accelometer.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
