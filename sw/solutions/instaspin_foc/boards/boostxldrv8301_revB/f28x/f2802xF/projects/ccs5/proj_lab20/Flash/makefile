################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.2.3

ORDERED_OBJS += \
"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/modules/fast/lib/32b/f28x/f2802x/2802xRevB_Fast_IQ_ROMSymbols.lib" \
"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/modules/fast/lib/32b/f28x/f2802x/2802xRevB_Fast_ONLY_ROMSymbols.lib" \
"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/modules/fast/lib/32b/f28x/f2802x/2802xRevB_Fast_RTS_ROMSymbols.lib" \
-l"rts2800_ml.lib" \
"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/modules/fast/lib/32b/f28x/f2802x/fast_public.lib" \
"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/modules/iqmath/lib/f28x/32b/IQmath.lib" \
"./CodeStartBranch.obj" \
"./adc.obj" \
"./clarke.obj" \
"./clk.obj" \
"./cpu.obj" \
"./ctrl.obj" \
"./drv8301.obj" \
"./filter_fo.obj" \
"./flash.obj" \
"./gpio.obj" \
"./hal.obj" \
"./ipark.obj" \
"./memCopy.obj" \
"./offset.obj" \
"./osc.obj" \
"./park.obj" \
"./pid.obj" \
"./pie.obj" \
"./pll.obj" \
"./proj_lab20.obj" \
"./pwm.obj" \
"./pwr.obj" \
"./spi.obj" \
"./svgen.obj" \
"./timer.obj" \
"./traj.obj" \
"./usDelay.obj" \
"./user.obj" \
"./wdog.obj" \
"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/ide/ccs/cmd/f2802x/F28027F.cmd" \
$(GEN_CMDS__FLAG) \
-lesmo.lib \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
proj_lab20.out \

EXE_OUTPUTS__QUOTED += \
"proj_lab20.out" \


# All Target
all: proj_lab20.out

# Tool invocations
proj_lab20.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.2.3/bin/cl2000" -v28 -ml -mt -O2 -g --define=FLASH --diag_warning=225 --display_error_number --asm_listing -z -m"proj_lab20.map" --stack_size=0x200 --warn_sections -i"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.2.3/lib" -i"D:/victor TI/Customer issue/Zero/4inOne/Code/sw/eSMO/v100/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-c2000_6.2.3/include" --reread_libs --priority --disable_auto_rts --diag_suppress=16002 --xml_link_info="proj_lab20_linkInfo.xml" --rom_model -o "proj_lab20.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "adc.pp" "clarke.pp" "clk.pp" "cpu.pp" "ctrl.pp" "drv8301.pp" "filter_fo.pp" "flash.pp" "gpio.pp" "hal.pp" "ipark.pp" "memCopy.pp" "offset.pp" "osc.pp" "park.pp" "pid.pp" "pie.pp" "pll.pp" "proj_lab20.pp" "pwm.pp" "pwr.pp" "spi.pp" "svgen.pp" "timer.pp" "traj.pp" "user.pp" "wdog.pp" 
	-$(RM) "CodeStartBranch.obj" "adc.obj" "clarke.obj" "clk.obj" "cpu.obj" "ctrl.obj" "drv8301.obj" "filter_fo.obj" "flash.obj" "gpio.obj" "hal.obj" "ipark.obj" "memCopy.obj" "offset.obj" "osc.obj" "park.obj" "pid.obj" "pie.obj" "pll.obj" "proj_lab20.obj" "pwm.obj" "pwr.obj" "spi.obj" "svgen.obj" "timer.obj" "traj.obj" "usDelay.obj" "user.obj" "wdog.obj" 
	-$(RM) "CodeStartBranch.pp" "usDelay.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

