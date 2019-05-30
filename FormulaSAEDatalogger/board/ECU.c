/******************************************************************************
*   ECU.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The ECU module of the Formula SAE Datalogger initializes and establishes
*   a CAN communication link with the vehicle's ECU,
*
**   The Accel/Gyro module of the Formula SAE Datalogger initializes and
*   establishes an I2C communication link with the on-board accelerometer and
*   gyroscope unit. The relevant data is then read from the unit and made
*   available other peripherals.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/29/2019
*
*   Created on: 05/25/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "ECU.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Debug.h"
#include "fsl_flexcan.h"

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/
#define ENABLE                          0x01U
#define CAN0_SRC_CLK_HZ             60000000U
#define RX_MESSAGE_BUFFER_NUM              9U
#define TX_MESSAGE_BUFFER_NUM              8U
#define CAN_FRAME_PAYLOAD_BYTES            8U
#define ALT_2_CAN                       0x02U
#define CAN0_BAUDRATE                 500000U

#define ECUINPUTTASK_PRIORITY              5U
#define ECUINPUTTASK_STKSIZE             256U

#define ECUOUTPUTTASK_PRIORITY             5U
#define ECUOUTPUTTASK_STKSIZE            256U

/* PORTB */
#define CAN0_TX_PIN_NUM                  18U
#define CAN0_RX_PIN_NUM                  19U

/* PRESCALER = 60MHz/(500kHz * TimeQuanta)
 * TimeQuanta = (PSEG1 + PSEG2 + PROPSEG) + 4 */
#define PRESCALER                        12U
#define PSEG1                             3U
#define PSEG2                             2U
#define PROPSEG                           1U

/*******************************************************************************
 * Private Variables
 ******************************************************************************/
flexcan_handle_t ecuCANHandle;
flexcan_mb_transfer_t ecuTxXfer, ecuRxXfer;
flexcan_frame_t ecuTxFrame, ecuRxFrame;

/* Primitives come from API. */
static volatile bool txComplete = false;
static volatile bool rxComplete = false;

static SemaphoreHandle_t ecuCurrentDataKey;

static TaskHandle_t ecuInputTaskHandle = NULL;
static TaskHandle_t ecuOutputTaskHandle = NULL;

static uint32_t ecuCurrentData[2];

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void ecuCallbackISR(CAN_Type *, flexcan_handle_t *, status_t , uint32_t, void *);

static void ecuInputTask(void *);

static void ecuOutputTask(void *);

/******************************************************************************
*   ECUInit() - Public function to configure CAN0 for both Rx and Tx capability.
*   Uses fsl_flexcan.h device driver API for easy configuration. CAN0 is set
*   to 500kHz, with a 8 byte Rx and 8 byte Tx buffer. CAN0 IRQs are set to
*   priority 2 for FreeRTOS capability.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void ECUInit()
{
    BaseType_t task_create_return;
    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;

    /* Enable PORTB and CAN0 clocks */
    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);
    SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0(ENABLE);

    /* Port muxing for CAN0 pins */
    PORTB->PCR[CAN0_TX_PIN_NUM] = PORT_PCR_MUX(ALT_2_CAN);
    PORTB->PCR[CAN0_RX_PIN_NUM] = PORT_PCR_MUX(ALT_2_CAN);

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
    flexcanConfig.baudRate = CAN0_BAUDRATE;
    flexcanConfig.enableLoopBack = true;

    FLEXCAN_Init(CAN0, &flexcanConfig, CAN0_SRC_CLK_HZ);

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type = kFLEXCAN_FrameTypeData;
    mbConfig.id = FLEXCAN_ID_STD(0x45);

    FLEXCAN_SetRxMbConfig(CAN0, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

    FLEXCAN_SetTxMbConfig(CAN0, TX_MESSAGE_BUFFER_NUM, true);

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(CAN0, &ecuCANHandle, ecuCallbackISR, NULL);

    /* Start receive data through Rx Message Buffer. */
    ecuRxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
    ecuRxXfer.frame = &ecuRxFrame;

    /* Prepare Tx Frame for sending. */
    ecuTxFrame.format = kFLEXCAN_FrameFormatStandard;
    ecuTxFrame.type = kFLEXCAN_FrameTypeData;
    ecuTxFrame.id = FLEXCAN_ID_STD(0x45);
    ecuTxFrame.length = CAN_FRAME_PAYLOAD_BYTES;

    /* Send data through Tx Message Buffer. */
    ecuTxXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    ecuTxXfer.frame = &ecuTxFrame;

    ecuCurrentDataKey = xSemaphoreCreateMutex();
    while(ecuCurrentDataKey == NULL){ /* Error trap */ }

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(ecuInputTask,
                                     "ECU CAN Input Task",
                                     ECUINPUTTASK_STKSIZE,
                                     NULL,
                                     ECUINPUTTASK_PRIORITY,
                                     &ecuInputTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(ecuOutputTask,
                                     "ECU CAN Output Task",
                                     ECUOUTPUTTASK_STKSIZE,
                                     NULL,
                                     ECUOUTPUTTASK_PRIORITY,
                                     &ecuOutputTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, 2U);
    NVIC_SetPriority(CAN0_Bus_Off_IRQn, 2U);
    NVIC_SetPriority(CAN0_Error_IRQn, 2U);
    NVIC_SetPriority(CAN0_Tx_Warning_IRQn, 2U);
    NVIC_SetPriority(CAN0_Rx_Warning_IRQn, 2U);
    NVIC_SetPriority(CAN0_Wake_Up_IRQn, 2U);
}

/******************************************************************************
*   ecuInputTask() - This FreeRTOS task consists of framework to receive data
*   on CAN0 in non-blocking, FreeRTOS friendly operation.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
******************************************************************************/
static void ecuInputTask(void *pvParameters)
{
    while(1)
    {
        FLEXCAN_TransferReceiveNonBlocking(CAN0, &ecuCANHandle, &ecuRxXfer);

        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        /* Pend on Mutex to update data structure */
        xSemaphoreTake(ecuCurrentDataKey, portMAX_DELAY);

        ecuCurrentData[0] = ecuRxFrame.dataWord0;
        ecuCurrentData[1] = ecuRxFrame.dataWord1;

        xSemaphoreGive(ecuCurrentDataKey);
    }
}


/******************************************************************************
*   ecuOutputTask() - This FreeRTOS task consists of framework to transmit data
*   on CAN0 in non-blocking, FreeRTOS friendly operation.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
******************************************************************************/
static void ecuOutputTask(void *pvParameters)
{
    while(1)
    {
        ecuTxFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
                               CAN_WORD0_DATA_BYTE_3(0x44);

        ecuTxFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
                               CAN_WORD1_DATA_BYTE_7(0x88);

        FLEXCAN_TransferSendNonBlocking(CAN0, &ecuCANHandle, &ecuTxXfer);

        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(5000U));
    }
}


/******************************************************************************
*   ecuCallbackISR() - This callback function is called by
*   FLEXCAN_TransferHandleIRQ within the CAN driver API, and thus can occur
*   asynchronously from Formula SAE Datalogger program flow. Called whenever
*   a CAN0 Tx finished or Rx started interrupt occurs.
*
*   Parameters:
*
*       The parameters of this function are only specified within fsl_flexcan.c,
*       meaning this function should never be called within this module.
*
*   Return: None
******************************************************************************/
static void ecuCallbackISR(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    switch (status)
    {
        /* Process FlexCAN Rx event. */
        case kStatus_FLEXCAN_RxIdle:
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
                rxComplete = true;
                vTaskNotifyGiveFromISR(ecuInputTaskHandle, &xHigherPriorityTaskWoken);
            }
            break;

        /* Process FlexCAN Tx event. */
        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                txComplete = true;
                vTaskNotifyGiveFromISR(ecuOutputTaskHandle, &xHigherPriorityTaskWoken);
            }
            break;

        default:
            break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

