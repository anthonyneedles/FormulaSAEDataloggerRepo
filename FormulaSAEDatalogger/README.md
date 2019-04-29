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