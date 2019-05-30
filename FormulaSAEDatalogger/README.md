#FormulaSAEDataloggerRepo
The Formula SAE Datalogger is my senior capstone project, which takes places over three quarters: Fall 2018 (project proposals and requirement finalization), Winter 2019 (hardware development), and Spring 2019 (software development). This product is being made for the WWU SAE Formula team. The Formula SAE Datalogger will store all required on board sensor data for the Formula SAE team vehicle, as well provide the telemetry and data acquisition units access to real-time data. The source of the data will either be from the ECU or DAQ CAN networks or analog inputs that can accept the standardized vehicle sensor output ranges. In order to facilitate sensor data analysis in respect to a specific time, real time timestamps will be generated for each data sample. These sets of timestamps and sensor data will be stored in non-volatile memory in the form of an SD card so that the Formula SAE team can easily remove the stored data for analysis on a computer. Data will also be available for communication to an external, separate wireless telemetry for real-time data acquisition. Having sensor data available in real time, as well as having access to timestamped data, is a very useful tool for the Formula SAE team in optimizing vehicle performance and maintenance.

Changelog:

v0.0.0
Generated bare Amazon FreeRTOS (version 10.0.1) project from custom MK66FN2M0xxx18 SDK (SDK version 2.5.0, manifest version 3.4.0).

v0.1.0
Added FreeRTOS + FAT extension, added both FreeRTOS and FreeRTOS + FAT config files (FreeRTOSConfig.h and FreeRTOSFATDefaults.h) to board directory as they are application specific.

v0.2.0
Initial bulk completion of clock configuration module. In-line and function comments completed. Validation and verification of module still needed. 

v0.3.0
Started basic UART communication for GPS module. Since this is just to establish communications, writing a generic UART driver will be put off until later in the software development stage.

v0.4.0
Completed clock configuration. Hardware testing completed via CLKOUT and RTC_CLKOUT pins. System oscillator 12 MHz frequency, 1 Hz RTC counting, 60 MHz Flexbus clock directly observed and verified. Debugged previous hard faulting. Updated all comments in ClockConfig.c/.h.

v0.5.0
Completed writing initial Digital Output module. Tested and verified proper operation of code on hardware. More will need to be done for new board revision. Most code in FormulaSAEDatalogger.c is for testing.

v0.5.1
Completed Digital Output module for complete operation. Still need to test and verify in hardware. Added functionality through new private structure in DigitalOutput.c to handle pins of different PORT and GPIO block, such as PTA vs PTB. Any DOUT power port/gpio/pin and state port/gpio/pin change be changed to any other valid MK66F18 port/gpio/pin with no additional changes except ensuring new port clock is enabled in DigitalOutputInit().

v0.5.2
Completed new Digital Output module that is faster, smaller, and easier to read. DigitalOuput_GENERIC (previous Digital Output module) allows dynamically changing ports/pins, but since the DOUT ports/pins are fixed this is unnecessary and provides a complex solution to a simple problem. Still need to test and verify in hardware.

v0.6.0
Started Analog Input module. Established initial public setting function. Have yet to test in hardware. Changed naming conventions for DigitalOutput.c/.h and ClockConfig.c/.h: public function/variable names start with mixed case module abbreviation, while private function/variable names start with lower case module abbreviation. This will be the norm as development continues.

v0.6.1
Nearly completed writing Analog Input module, including ADC/PIT initialization. Tested power and conditioning section in hardware, as well as observing PIT1 frequency (8kHz). Need to test ADC in hardware. Updated most comments in Analog In module. 

v0.6.1
Nearly completed writing Analog Input module, including ADC/PIT initialization. Tested power and conditioning section in hardware, as well as observing PIT1 frequency (8kHz). Need to test ADC in hardware. Updated most comments in Analog In module.

v0.6.2
Analog Input module almost complete, hardware testing proves ADC1 is triggering correctly, but measured data does not change. NOTE: IRQs NEED TO MANUALLY BE GIVEN A PRIORITY <=2 SO THAT FREERTOS CAN MASK THEM IN CRITICAL SECTIONS.

v0.7.0
Need to test ADC in hardware. Created GPS module to receive time and date data from GPS NMEA 1Hz stream on UART4. Need to test in hardware.

v0.7.1
Need to test ADC in hardware. Need to test GPS UART in hardware. Completed initialization and basic use of fsl FreeRTOS I2C driver. Need to test in hardware. Updated Analog Input and Digital Output modules to change how setting/clearing was handled for power/state/conditioning pin selects.

v0.7.2
Analog Input testing provided fault with 12V conditioning, most likely a hardware issue. ADC has not been hardware tested. GPS NMEA UART data successfully parsed, except GPS can not fix inside of the lab, so I will need to test it outside. Successfully communicated with Accel/Gyro via I2C, need to integrate with FreeRTOS task.

v0.8.0
After many attempts, I have successfully completed a non-blocking, FreeRTOS friendly I2C driver that has been tested to read configure the accelerometer and gyroscope, as well as read and save all sensor data. Comments for the Accel/Gyro module are not yet completely updated.

v0.9.0
Updated comments and refactored some variable names in AccelGyro, DigitalOutput, GPS, and Analog In modules. Completely finished AccelGyro module. Created Debug module for setting/clearing a GPIO for precise timing measurements. Added 10ms incrementer task in GPS for timestamps. Created Telemetry task, started on Telemetry output task, need to verify UART output in hardware. Need to verify Analog input module.  

v1.0.0 - SOFT RELEASE 
Completed framework for Telemetry module. Both reception and transmission over UART3 with non-blocking, FreeRTOS friendly UART driver confirmed in hardware. Need to confirm data frame content with wireless telemetry unit for further development of Telemetry module.

v1.0.1 
Need to confirm data frame content with wireless telemetry unit for further development of Telemetry module. Completed framework for ECU/DAQ CAN modules. Confirmed transmission/reception over CAN network at 500kb/s using a FreeRTOS friendly implementation of non-blocking fsl CAN driver API. Need to confirm data frame content with ECU and DAQ for further development of ECU and DAQ module.

v1.1.0 
Confirmed frame content with wireless telemetry unit. Further development on CAN modules needed. Confirmed complete telemetry recpetion of expected frame with acknowledge, as well as dout acceptance. All multitasking confirmed in hardware to operate peacefully. Developed error handling method of resurrecting modules. Need to verify and finish Analog Input module. 