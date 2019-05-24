/******************************************************************************
*   Telemetry.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Telemetry module of the Formula SAE Datalogger initializes UART3 for
*   communication to and from the wireless telemetry unit. At 2Hz, a package of
*   requested data is sent, along with a time stamp. In addition, at 2Hz,
*   configuration data is received by the wireless telemetry unit.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/23/2019
*
*   Created on: 05/20/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "AccelGyro.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "AccelGyro.h"
#include "GPS.h"
#include "DigitalOutput.h"
#include "AnalogInput.h"
#include "Debug.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                          0x01U
#define ALT_3_UART                      0x03U
#define SET                                1U
#define BAUD_RATE_DIV                   0x20U
#define BAUD_RATE_FA                    0x12U
#define SINGLE_STOP_BIT                    0U
#define NO_PARITY_BIT                      0U
#define EIGHT_BIT_MODE                     0U
#define PIT_100HZ_LDVAL               599999U
#define PIT0                            0x00U

#define TELINPUTTASK_PRIORITY              4U
#define TELINPUTTASK_STKSIZE             256U
#define TELOUTPUTTASK_PRIORITY             5U
#define TELOUTPUTTASK_STKSIZE           1024U

/* PORTB. */
#define UART3_RX_PIN_NUM                  10U
#define UART3_TX_PIN_NUM                  11U

/* Status flags. */
#define UART3_TDRE_FLAG (((UART3->S1) & UART_S1_TDRE_MASK) >> UART_S1_TDRE_SHIFT)
#define UART3_RDRF_FLAG (((UART3->S1) & UART_S1_RDRF_MASK) >> UART_S1_RDRF_SHIFT)

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Task handle for Accel/Gyro sampler task, used by I2C3_IRQHandler() to post
 * task notification. */
static TaskHandle_t telInputTaskHandle = NULL;

/* Task handle for Accel/Gyro sampler task, used by I2C3_IRQHandler() to post
 * task notification. */
static TaskHandle_t telOutputTaskHandle = NULL;

static ag_data_t telAGData;
static gps_data_t telGPSData;
static ain_data_t telAInData;
static uint8_t telRxBuffer[50];
static uint8_t telTxByteToSend;
static uint8_t telRxBufferIndex;


/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
static void telInputTask(void *pvParameters);

static void telOutputTask(void *pvParameters);

static void telPendOnInterrupt(void);

static void telSendTime(void);

static void telSendAG(void);

static void telSendAIn(void);

/******************************************************************************
*   TelInit() - Public function to configure UART3 for reception and
*   transmission with the separate wireless telemetry unit.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void TelInit()
{
    BaseType_t task_create_return;

    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);
    SIM->SCGC4 |= SIM_SCGC4_UART3(ENABLE);

    PORTB->PCR[UART3_RX_PIN_NUM] = PORT_PCR_MUX(ALT_3_UART);
    PORTB->PCR[UART3_TX_PIN_NUM] = PORT_PCR_MUX(ALT_3_UART);

    /* Baud rate = Module clock / (16 × (BAUD_RATE_DIV + BAUD_RATE_FA))
     * Module clock = 60MHz, BRD = 32, BRFA = 0.5625
     * Baud rate = ~115163 */
    UART3->BDH = (uint8_t)(BAUD_RATE_DIV >> 8);
    UART3->BDL = (uint8_t)(BAUD_RATE_DIV);
    UART3->C4 = (uint8_t)(BAUD_RATE_FA);

    UART3->C1 = (UART_C1_PE(NO_PARITY_BIT) | UART_C1_M(EIGHT_BIT_MODE));

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(telInputTask,
                                     "Telemetry Input Task",
                                     TELINPUTTASK_STKSIZE,
                                     NULL,
                                     TELINPUTTASK_PRIORITY,
                                     &telInputTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(telOutputTask,
                                     "Telemetry Output Task",
                                     TELOUTPUTTASK_STKSIZE,
                                     NULL,
                                     TELOUTPUTTASK_PRIORITY,
                                     &telOutputTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    NVIC_SetPriority(UART3_RX_TX_IRQn, 2U);
    NVIC_ClearPendingIRQ(UART3_RX_TX_IRQn);
    NVIC_EnableIRQ(UART3_RX_TX_IRQn);
}

/******************************************************************************
*   telInputTask() - This FreeRTOS task consists of sequential operation of
*   saving all user-input configuration data transmitted from wireless telemetry
*   via UART3. A non-blocking, FreeRTOS friendly UART driver will be used to
*   pend on a new byte reception. While this should be occurring at 2Hz, exact
*   timing is dependent on telemetry unit.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
******************************************************************************/
static void telInputTask(void *pvParameters)
{
    UART3->C2 |= UART_C2_RE(ENABLE);
    telRxBufferIndex = 0;

    while(1)
    {
        UART3->C2 |= UART_C2_RIE(ENABLE);
        while(telRxBufferIndex < 27)
        {
            telPendOnInterrupt();
            telRxBufferIndex++;
        }
        UART3->C2 &= ~UART_C2_RIE(ENABLE);
        telRxBufferIndex = 0;
    }
}

/******************************************************************************
*   telOutputTask() - This FreeRTOS task consists of sequential operation of
*   grabbing the current data in all modules, then transmitting the data over
*   UART3 to the wireless telemetry unit with a non-blocking, FreeRTOS friendly
*   UART driver. This task repeats operation at 2Hz.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
******************************************************************************/
static void telOutputTask(void *pvParameters)
{
    UART3->C2 |= UART_C2_TE(ENABLE);

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500U));

        /* Grabbing all data. */
        GPSGetData(&telGPSData);
        AGGetData(&telAGData);
        AInGetData(&telAInData);

        /* Sending data. */
        telSendTime();
        telSendAG();
        telSendAIn();
    }
}

/******************************************************************************
*   UART3_RX_TX_IRQHandler() - Interrupt handler for UART3 TDRE and RDRF flags.
*   This triggers upon both empty transmit buffer and full receive buffer.
*   Clears TDRE flag by reading UART3 S1 register and writing to the UART3 D
*   register. Clears RDRF by reading UART3 S1 register and reading from the
*   UART3 D register. Posts task notification to corresponding Telemetry
*   Input/Output Task. This allows non-blocking UART driver operation.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void UART3_RX_TX_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(UART3_TDRE_FLAG == SET)
    { /* Sending data, Tx buffer is empty. */
        UART3->D = telTxByteToSend;
        vTaskNotifyGiveFromISR(telOutputTaskHandle, &xHigherPriorityTaskWoken);
    } else {}

    if(UART3_RDRF_FLAG == SET)
    { /* Reading data, Rx buffer is full. */
        telRxBuffer[telRxBufferIndex] = UART3->D;
        vTaskNotifyGiveFromISR(telInputTaskHandle, &xHigherPriorityTaskWoken);
    } else {}

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   telPendOnInterrupt() - Private function that pends on the UART3 ISR. This
*   will both when the UART3 TX buffer is empty and when the UART3 RX buffer
*   is full. The ISR will check which flag triggered the interrupt, and notifies
*   the corresponding task. This ISR allows the telemetry tasks to enter an
*   idle state and allow the CPU to active other tasks.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void telPendOnInterrupt()
{
    uint8_t notify_count;

    /* Place task into idle state until I2C3 ISR notifies task. */
    notify_count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if(notify_count == 0)
    { /* If this takes longer than 1ms, go to ERROR state. */
    } else {}
}

/******************************************************************************
*   telSendTime() - Private function transmits captured GPS data by setting
*   global variable that will be written to UART3 data register upon transmit
*   buffer full interrupt. Non-blocking and FreeRTOS friendly.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void telSendTime()
{
    telTxByteToSend = telGPSData.year;
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telPendOnInterrupt();
    telTxByteToSend = telGPSData.month;
    telPendOnInterrupt();
    telTxByteToSend = telGPSData.day;
    telPendOnInterrupt();
    telTxByteToSend = telGPSData.hour;
    telPendOnInterrupt();
    telTxByteToSend = telGPSData.min;
    telPendOnInterrupt();
    telTxByteToSend = telGPSData.sec;
    telPendOnInterrupt();
    telTxByteToSend = telGPSData.ms;
    telPendOnInterrupt();
    UART3->C2 &= ~UART_C2_TIE(ENABLE);
}

/******************************************************************************
*   telSendAG() - Private function transmits captured Accel/Gyro data. 16 bit
*   values are sent upper byte first, then lower byte. Non-blocking and
*   FreeRTOS friendly.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void telSendAG()
{
    telTxByteToSend = ((uint8_t)(telAGData.accel_data.x >> 8));
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.accel_data.x);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.accel_data.y >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.accel_data.y);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.accel_data.z >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.accel_data.z);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.gyro_data.x >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.gyro_data.x);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.gyro_data.y >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.gyro_data.y);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.gyro_data.z >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.gyro_data.z);
    telPendOnInterrupt();
    UART3->C2 &= ~UART_C2_TIE(ENABLE);
}

/******************************************************************************
*   telSendAG() - Private function transmits captured Accel/Gyro data by setting
*   global variable that will be written to UART3 data register upon transmit
*   buffer full interrupt. Non-blocking and FreeRTOS friendly.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void telSendAIn()
{
    telTxByteToSend = ((uint8_t)(telAInData.ain1_data >> 8));
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain1_data);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAInData.ain2_data >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain2_data);
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAInData.ain3_data >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain3_data);;
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)(telAInData.ain4_data >> 8));
    telPendOnInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain4_data);
    telPendOnInterrupt();
    UART3->C2 &= ~UART_C2_TIE(ENABLE);
}
