################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -O0 --opt_for_speed=2 --fp_reassoc=on --include_path="C:/CodeComposerStudio/workspace_v9/FIR_FILTER_PROJ" --include_path="C:/tidcs/c28/DSP2833x/v131/DSP2833x_headers/include" --include_path="C:/tidcs/c28/DSP2833x/v131/DSP2833x_common/include" --include_path="C:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/include" --advice:performance=all -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/bin/cl2000" -v28 -ml -mt --float_support=fpu32 -O0 --opt_for_speed=2 --fp_reassoc=on --include_path="C:/CodeComposerStudio/workspace_v9/FIR_FILTER_PROJ" --include_path="C:/tidcs/c28/DSP2833x/v131/DSP2833x_headers/include" --include_path="C:/tidcs/c28/DSP2833x/v131/DSP2833x_common/include" --include_path="C:/ti/ccs930/ccs/tools/compiler/ti-cgt-c2000_18.12.4.LTS/include" --advice:performance=all -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


