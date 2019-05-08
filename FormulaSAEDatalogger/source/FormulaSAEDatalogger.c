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
#include "semphr.h"
#include "ClockConfig.h"
#include "DigitalOutput.h"
#include "AnalogInput.h"

//void ClockInitTask(void *pvParameters);

void main(void) {
    volatile uint32_t test_int = 0;
//    DigOutMsg_t test_msg_dout = {0xAA, 0xAA};
//    AnlgInMsg_t test_msg_ain = {0x5B, 0x55AA55AA};

//    DigOutInit();
//    DigOutSet(test_msg_dout);
//    test_msg_dout.power_field = 0x55;
//    test_msg_dout.state_field = 0x55;
//    DigOutSet(test_msg_dout);

//    AnlgInSet(test_msg_ain);
//    test_msg_ain.power_state_field = 0x55;
//    test_msg_ain.sampling_rate_field = 0xAA55AA55;
//    AnlgInSet(test_msg_ain);

//    xTaskCreate(ClockInitTask,
//                "Clock Init Task",
//                256,
//                NULL,
//                1,
//                NULL);

    ClkCfgRun();


    AnlgInInit();


    vTaskStartScheduler();
    while(1){
        test_int++;
    }
}

//void ClockInitTask(void *pvParameters)
//{
//    volatile uint32_t test_int = 0;
//    while(1)
//    {
//        while(1){
//            test_int++;
//        }
//    }
//    vTaskDelete(NULL);
//}
