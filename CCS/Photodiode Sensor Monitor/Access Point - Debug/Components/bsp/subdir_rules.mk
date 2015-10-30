################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Components/bsp/bsp.obj: ../Components/bsp/bsp.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"c:/ti/ccsv6/tools/compiler/msp430_4.3.5/bin/cl430" --cmd_file="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/peer applications/Configuration/smpl_nwk_config.dat" --cmd_file="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/peer applications/Configuration/Access Point/smpl_config.dat"  -vmsp -g --include_path="c:/ti/ccsv6/tools/compiler/msp430_4.3.5/include" --include_path="c:/ti/ccsv6/ccs_base/msp430/include" --include_path="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/Components/simpliciti/nwk" --include_path="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/Components/simpliciti/network_applications" --include_path="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/Components/mrfi" --include_path="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/Components/bsp/drivers" --include_path="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/Components/bsp/boards/EZ430RF" --include_path="C:/Users/Ralph S-J/Dropbox/02 UNIVERSITY/02 Technical/03 sensor_monitor/photodiode/CCS/Photodiode Sensor Monitor/Components/bsp" --define=MRFI_CC2500 --printf_support=full --preproc_with_compile --preproc_dependency="Components/bsp/bsp.pp" --obj_directory="Components/bsp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


