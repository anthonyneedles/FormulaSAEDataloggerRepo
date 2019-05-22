/******************************************************************************
*   Telemetry.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*

*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/20/2019
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

/* Task parameters */
#define TELINPUTTASK_PRIORITY              4U
#define TELINPUTTASK_STKSIZE             256U

#define TELOUTPUTTASK_PRIORITY             5U
#define TELOUTPUTTASK_STKSIZE           1024U

/* UART3 pin numbers, all PORTB. */
#define UART3_RX_PIN_NUM                  10U
#define UART3_TX_PIN_NUM                  11U

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
static ain_data_t telAINData;


/******************************************************************************
*   Private Function Prototypes
******************************************************************************/

static void telInputTask(void *pvParameters);

static void telOutputTask(void *pvParameters);

static void telPendOnInterrupt(void);

static void telSendTime(void);

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

    /* No parity bit, eight bit mode */
    UART3->C1 = (UART_C1_PE(NO_PARITY_BIT) | UART_C1_M(EIGHT_BIT_MODE));

//    /* Creation of accelerometer/gyroscope sampling task. */
//    task_create_return = xTaskCreate(telInputTask,
//                                     "Telemetry Input Task",
//                                     TELINPUTTASK_STKSIZE,
//                                     NULL,
//                                     TELINPUTTASK_PRIORITY,
//                                     &telInputTaskHandle);
//
//    while(task_create_return == pdFAIL){ /* Error trap */ }

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

static void telInputTask(void *pvParameters)
{
    UART3->C2 |= (UART_C2_RIE(ENABLE) | UART_C2_RE(ENABLE));

    while(1)
    {

    }
}

static void telOutputTask(void *pvParameters)
{
    UART3->C2 |= UART_C2_TE(ENABLE);

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500U));

        GPSGetData(&telGPSData);
//        AINGetData(&telAINData);
        AGGetData(&telAGData);

        UART3->C2 |= UART_C2_TIE(ENABLE);
//        telSendTime();
        telPendOnInterrupt();
        UART3->D = telGPSData.year;
        telPendOnInterrupt();
        UART3->D = telGPSData.month;
        telPendOnInterrupt();
        UART3->D = telGPSData.day;
        telPendOnInterrupt();
        UART3->D = telGPSData.hour;
        telPendOnInterrupt();
        UART3->D = telGPSData.min;
        telPendOnInterrupt();
        UART3->D = telGPSData.sec;
        telPendOnInterrupt();
        UART3->D = telGPSData.ms;
        UART3->C2 &= ~UART_C2_TIE(ENABLE);



    }
}

//static void telSendTime()
//{
//
//}

void UART3_RX_TX_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(UART3_TDRE_FLAG == SET)
    {
        vTaskNotifyGiveFromISR(telInputTaskHandle, &xHigherPriorityTaskWoken);
    } else {}

    if(UART3_RDRF_FLAG == SET)
    {
        vTaskNotifyGiveFromISR(telOutputTaskHandle, &xHigherPriorityTaskWoken);
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
    notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1U));
    if(notify_count == 0)
    { /* If this takes longer than 1ms, go to ERROR state. */
//        agI2CState = ERROR;
    } else {}
}
