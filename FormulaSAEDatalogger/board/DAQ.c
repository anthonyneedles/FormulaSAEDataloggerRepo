/******************************************************************************
*   DAQ.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 06/03/2019
*
*   Created on: 06/03/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "DAQ.h"
#include "GPS.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Debug.h"
#include "fsl_flexcan.h"

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/
#define ENABLE                          0x01U
#define CAN1_SRC_CLK_HZ             60000000U
#define RX_MB_NUM                          9U
#define TX_MB_NUM                          8U
#define CAN_FRAME_PAYLOAD_BYTES            8U
#define ALT_2_CAN                       0x02U
#define CAN1_BAUDRATE                 500000U

#define DAQINPUTTASK_PRIORITY              8U
#define DAQINPUTTASK_STKSIZE             256U

#define DAQOUTPUTTASK_PRIORITY             2U
#define DAQOUTPUTTASK_STKSIZE            256U

/* PORTC */
#define CAN1_TX_PIN_NUM                  17U
#define CAN1_RX_PIN_NUM                  16U

/* PRESCALER = 60MHz/(500kHz * TimeQuanta)
 * TimeQuanta = (PSEG1 + PSEG2 + PROPSEG) + 4 */
#define PRESCALER                        12U
#define PSEG1                             3U
#define PSEG2                             2U
#define PROPSEG                           1U

#define DAQ_ID FLEXCAN_ID_STD(0x80U)
#define SELF_ID FLEXCAN_ID_STD(0x81U)

#define TIME_PREAMBLE                  0xAAU
#define CONFIG_PREAMBLE                0x55U

#define RX_MESSAGE_BUFFER_NUM 9U
#define TX_MESSAGE_BUFFER_NUM 8U

#define MODULE_ADDR_BYTE           dataByte0
#define SENSOR_ADDR_BYTE           dataByte1
#define TIME_SEC_BYTE              dataByte2
#define TIME_CS_BYTE               dataByte3
#define DATA_UPPER_BYTE            dataByte4
#define DATA_LOWER_BYTE            dataByte5

#define SENSOR1_POW_MASK     ((uint8_t)0x01U)
#define SENSOR1_POW_SHIFT       ((uint8_t)0U)
#define FIND_SENSOR1_POW(msg) ((uint8_t)((msg & SENSOR1_POW_MASK) >> SENSOR1_POW_SHIFT))

#define SENSOR2_POW_MASK     ((uint8_t)0x02U)
#define SENSOR2_POW_SHIFT       ((uint8_t)1U)
#define FIND_SENSOR2_POW(msg) ((uint8_t)((msg & SENSOR2_POW_MASK) >> SENSOR2_POW_SHIFT))

#define SENSOR3_POW_MASK     ((uint8_t)0x04U)
#define SENSOR3_POW_SHIFT       ((uint8_t)2U)
#define FIND_SENSOR3_POW(msg) ((uint8_t)((msg & SENSOR3_POW_MASK) >> SENSOR3_POW_SHIFT))

#define SENSOR4_POW_MASK     ((uint8_t)0x08U)
#define SENSOR4_POW_SHIFT       ((uint8_t)3U)
#define FIND_SENSOR4_POW(msg) ((uint8_t)((msg & SENSOR4_POW_MASK) >> SENSOR4_POW_SHIFT))

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
flexcan_handle_t daqCANHandle;
flexcan_mb_transfer_t daqTxXfer;
flexcan_mb_transfer_t daqRxXfer;
flexcan_frame_t daqTxFrame;
flexcan_frame_t daqRxFrame;

static SemaphoreHandle_t daqCurrentDataKey;

static TaskHandle_t daqInputTaskHandle = NULL;
static TaskHandle_t daqOutputTaskHandle = NULL;

static daq_data_t daqCurrentData[8];
static gps_data_t daqGPSData;

static SemaphoreHandle_t daqInitConfigRecieved;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void daqCallbackISR(CAN_Type *, flexcan_handle_t *, status_t , uint32_t, void *);

static void daqInputTask(void *);

static void daqOutputTask(void *);

static void daqSendTime(void);

static void daqSendConfig(void);

/******************************************************************************
*   DAQInit() - Public function to configure CAN1 for both Rx and Tx capability.
*   Uses fsl_flexcan.h device driver API for easy configuration. CAN1 is set
*   to 500kHz, with a 8 byte Rx and 8 byte Tx buffer. CAN1 IRQs are set to
*   priority 2 for FreeRTOS capability.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void DAQInit()
{
    BaseType_t task_create_return;
    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;

    /* Enable PORTC and CAN1 clocks */
    SIM->SCGC5 |= SIM_SCGC5_PORTC(ENABLE);
    SIM->SCGC3 |= SIM_SCGC3_FLEXCAN1(ENABLE);

    /* Port muxing for CAN1 pins */
    PORTC->PCR[CAN1_TX_PIN_NUM] = PORT_PCR_MUX(ALT_2_CAN);
    PORTC->PCR[CAN1_RX_PIN_NUM] = PORT_PCR_MUX(ALT_2_CAN);

    /* Default Configuration:
     * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
     * flexcanConfig.baudRate = 1000000U;
     * flexcanConfig.baudRateFD = 2000000U;
     * flexcanConfig.maxMbNum = 16;
     * flexcanConfig.enableLoopBack = false;
     * flexcanConfig.enableSelfWakeup = false;
     * flexcanConfig.enableIndividMask = false;
     * flexcanConfig.enableDoze = false; */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);

    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
    flexcanConfig.baudRate = CAN1_BAUDRATE;
//    flexcanConfig.enableLoopBack = true;

    FLEXCAN_Init(CAN1, &flexcanConfig, CAN1_SRC_CLK_HZ);

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type = kFLEXCAN_FrameTypeData;
    mbConfig.id = DAQ_ID;

    FLEXCAN_SetRxMbConfig(CAN1, RX_MB_NUM, &mbConfig, true);

    FLEXCAN_SetTxMbConfig(CAN1, TX_MB_NUM, true);

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(CAN1, &daqCANHandle, daqCallbackISR, NULL);

    /* Start receive data through Rx Message Buffer. */
    daqRxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
    daqRxXfer.frame = &daqRxFrame;

    /* Prepare Tx Frame for sending. */
    daqTxFrame.format = kFLEXCAN_FrameFormatStandard;
    daqTxFrame.type = kFLEXCAN_FrameTypeData;
    daqTxFrame.id = SELF_ID;
    daqTxFrame.length = CAN_FRAME_PAYLOAD_BYTES;

    /* Send data through Tx Message Buffer. */
    daqTxXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    daqTxXfer.frame = &daqTxFrame;

    daqCurrentDataKey = xSemaphoreCreateMutex();
    while(daqCurrentDataKey == NULL){ /* Error trap */ }

    daqInitConfigRecieved = xSemaphoreCreateBinary();
    while(daqInitConfigRecieved == NULL){ /* Error trap */ }

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(daqInputTask,
                                     "DAQ CAN Input Task",
                                     DAQINPUTTASK_STKSIZE,
                                     NULL,
                                     DAQINPUTTASK_PRIORITY,
                                     &daqInputTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(daqOutputTask,
                                     "DAQ CAN Output Task",
                                     DAQOUTPUTTASK_STKSIZE,
                                     NULL,
                                     DAQOUTPUTTASK_PRIORITY,
                                     &daqOutputTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    NVIC_SetPriority(CAN1_ORed_Message_buffer_IRQn, 2U);
    NVIC_SetPriority(CAN1_Bus_Off_IRQn, 2U);
    NVIC_SetPriority(CAN1_Error_IRQn, 2U);
    NVIC_SetPriority(CAN1_Tx_Warning_IRQn, 2U);
    NVIC_SetPriority(CAN1_Rx_Warning_IRQn, 2U);
    NVIC_SetPriority(CAN1_Wake_Up_IRQn, 2U);
}

/******************************************************************************
*   daqInputTask() - This FreeRTOS task consists of framework to receive data
*   on CAN1 in non-blocking, FreeRTOS friendly operation. When a frame is
*   received, this task will parse the data and save the resulting data.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void daqInputTask(void *pvParameters)
{
    uint8_t sec;
    uint8_t centisec;
    uint16_t data;
    uint8_t module_num;
    uint8_t sensor_num;

    /* For testing only { */
    daq_msg_t msg;
    DAQSet(msg);
    /* } For testing only */

    while(1)
    {
        FLEXCAN_TransferReceiveNonBlocking(CAN1, &daqCANHandle, &daqRxXfer);

        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        module_num = daqRxFrame.MODULE_ADDR_BYTE;
        sensor_num = daqRxFrame.SENSOR_ADDR_BYTE;
        sec = daqRxFrame.TIME_SEC_BYTE;
        centisec = daqRxFrame.TIME_CS_BYTE;
        data = (((uint16_t)(daqRxFrame.DATA_UPPER_BYTE << 8U)) |
                ((uint16_t)(daqRxFrame.DATA_LOWER_BYTE)));

        if((module_num < 8) && (sensor_num < 4))
        {
            /* Pend on Mutex to update data structure */
            xSemaphoreTake(daqCurrentDataKey, portMAX_DELAY);

            daqCurrentData[module_num].sensor[sensor_num].time_sec = sec;
            daqCurrentData[module_num].sensor[sensor_num].time_cs = centisec;
            daqCurrentData[module_num].sensor[sensor_num].data = data;

            xSemaphoreGive(daqCurrentDataKey);
        } else
        {
            /* ERROR */
        }
    }
}


/******************************************************************************
*   daqOutputTask() - This FreeRTOS task consists of framework to transmit data
*   on CAN1 in non-blocking, FreeRTOS friendly operation. When the initial
*   config data is sent over the telemetry unit (only one, upon initialization)
*   this task will start by sending the initial config data to the DAQ units,
*   then send seconds and centi-seconds to each DAQ unit every 15 seconds.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void daqOutputTask(void *pvParameters)
{
    xSemaphoreTake(daqInitConfigRecieved, portMAX_DELAY);

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));
        daqSendConfig();
        daqSendTime();
    }
}


/******************************************************************************
*   daqCallbackISR() - This callback function is called by
*   FLEXCAN_TransferHandleIRQ within the CAN driver API, and thus can occur
*   asynchronously from Formula SAE Datalogger program flow. Called whenever
*   a CAN1 Tx finished or Rx started interrupt occurs.
*
*   Parameters:
*
*       The parameters of this function are only specified within fsl_flexcan.c,
*       meaning this function should never be called within this module.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void daqCallbackISR(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    switch (status)
    {
        /* Process FlexCAN Rx event. */
        case kStatus_FLEXCAN_RxIdle:
        case kStatus_FLEXCAN_RxOverflow:
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
                vTaskNotifyGiveFromISR(daqInputTaskHandle, &xHigherPriorityTaskWoken);
            }
            break;

        /* Process FlexCAN Tx event. */
        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                vTaskNotifyGiveFromISR(daqOutputTaskHandle, &xHigherPriorityTaskWoken);
            }
            break;

        default:
            break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   DAQSet() -
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void DAQSet(daq_msg_t msg)
{
    uint8_t temp_power;
    uint8_t temp_type;

    /* Pend on Mutex to update data structure */
    xSemaphoreTake(daqCurrentDataKey, portMAX_DELAY);

    for(uint8_t i = 0; i < 8; i++)
    {
        for(uint8_t j = 0; j < 4; j++)
        {
//            daqCurrentData[i].sensor[j].power = ((uint8_t)((msg.power_fields[i] & (uint8_t)(1U << j)) >> j));
            daqCurrentData[i].sensor[j].power = 0x01;
        }
    }

    for(uint8_t i = 0; i < 8; i++)
    {
        for(uint8_t j = 0; j < 4; j++)
        {

//            daqCurrentData[i].sensor[j].type = ((uint8_t)((msg.type_fields[i] & (uint8_t)(3U << (2U*j))) >> (2U*j)));
            daqCurrentData[i].sensor[j].type = 0x03;
        }
    }

    xSemaphoreGive(daqCurrentDataKey);
    xSemaphoreGive(daqInitConfigRecieved);
}

/******************************************************************************
*   DAQGetData() - Copies current DAQ sensor data to the passed structure array.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void DAQGetData(daq_data_t ldata[])
{
    /* Pend on Mutex to update data structure */
    xSemaphoreTake(daqCurrentDataKey, portMAX_DELAY);

    for(uint8_t i = 0; i < 8; i++)
    {
        ldata[i] = daqCurrentData[i];
    }

    xSemaphoreGive(daqCurrentDataKey);
}

/******************************************************************************
*   daqSendTime() - Loads CAN1 data buffer with current time frame and starts
*   CAN1 transfer of data.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void daqSendTime()
{
    GPSGetData(&daqGPSData);

    daqTxFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(TIME_PREAMBLE)  |
                           CAN_WORD0_DATA_BYTE_1(daqGPSData.sec) |
                           CAN_WORD0_DATA_BYTE_2(daqGPSData.cs)  |
                           CAN_WORD0_DATA_BYTE_3(0x11U);
    daqTxFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0xFFU) |
                           CAN_WORD1_DATA_BYTE_5(0xEEU) |
                           CAN_WORD1_DATA_BYTE_6(0xDDU) |
                           CAN_WORD1_DATA_BYTE_7(0xCCU);

    FLEXCAN_TransferSendNonBlocking(CAN1, &daqCANHandle, &daqTxXfer);

    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
}

/******************************************************************************
*   daqSendConfig() - Loads CAN1 data buffer with configuration frame and starts
*   CAN1 transfer of data. This is repeated for each DAQ.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void daqSendConfig()
{
    uint8_t power_field;

    for(uint8_t i = 0; i < 1; i++)
    {
        power_field = (daqCurrentData[i].sensor[SENSOR1].power) |
                      (daqCurrentData[i].sensor[SENSOR2].power << 1U) |
                      (daqCurrentData[i].sensor[SENSOR3].power << 2U) |
                      (daqCurrentData[i].sensor[SENSOR4].power << 3U);

        daqTxFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(CONFIG_PREAMBLE) |
                               CAN_WORD0_DATA_BYTE_1(i)               |
                               CAN_WORD0_DATA_BYTE_2(0x0F)     |
                               CAN_WORD0_DATA_BYTE_3(daqCurrentData[i].sensor[SENSOR1].type);
        daqTxFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(daqCurrentData[i].sensor[SENSOR2].type) |
                               CAN_WORD1_DATA_BYTE_5(daqCurrentData[i].sensor[SENSOR3].type) |
                               CAN_WORD1_DATA_BYTE_6(daqCurrentData[i].sensor[SENSOR4].type) |
                               CAN_WORD1_DATA_BYTE_7(0xCCU);

        FLEXCAN_TransferSendNonBlocking(CAN1, &daqCANHandle, &daqTxXfer);

        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    }
}
