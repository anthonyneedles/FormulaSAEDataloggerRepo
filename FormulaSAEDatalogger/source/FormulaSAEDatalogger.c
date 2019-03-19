/******************************************************************************
*   Formula SAE Datalogger.c v0.2.0
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Formula SAE Datalogger will store all required on board sensor data for
*   the Formula SAE team vehicle, as well provide the telemetry and data
*   acquisition units access to real-time data. The source of the data will
*   either be from the ECU or DAQ CAN networks or analog inputs that can accept
*   the standardized vehicle sensor output ranges. In order to facilitate sensor
*   data analysis in respect to a specific time, real time timestamps will be
*   generated for each data sample. These sets of timestamps and sensor data
*   will be stored in non-volatile memory in the form of an SD card so that the
*   Formula SAE team can easily remove the stored data for analysis on a
*   computer. Data will also be available for communication to an external,
*   separate wireless telemetry for real-time data acquisition. Having sensor
*   data available in real time, as well as having access to timestamped data,
*   is a very useful tool for the Formula SAE team in optimizing vehicle
*   performance and maintenance.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Created on: 03/12/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ClockConfig.h"


int main(void) {

    void ClockConfigRun(void);

    return 0 ;
}
