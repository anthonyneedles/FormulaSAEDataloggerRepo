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
*   Comments up to date as of: 05/29/2019
*
*   Created on: 05/20/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "Telemetry.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "AccelGyro.h"
#include "GPS.h"
#include "DigitalOutput.h"
#include "AnalogInput.h"
#include "DAQ.h"
#include "Debug.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                          0x01U
#define ENABLED                         0x01U
#define DISABLED                        0x00U
#define FALSE                           0x00U
#define TRUE                            0x01u
#define ALT_3_UART                      0x03U
#define SET                                1U
#define BAUD_RATE_DIV                   0x20U
#define BAUD_RATE_FA                    0x12U
#define SINGLE_STOP_BIT                    0U
#define NO_PARITY_BIT                      0U
#define EIGHT_BIT_MODE                     0U
#define PIT_100HZ_LDVAL               599999U
#define PIT0                            0x00U
#define CONFIG_FRAME_DATA_BYTES           15U
#define DOUT_FRAME_DATA_BYTES              2U

#define TELINPUTTASK_PRIORITY              3U
#define TELINPUTTASK_STKSIZE            1024U
#define TELOUTPUTTASK_PRIORITY             4U
#define TELOUTPUTTASK_STKSIZE           1024U

/* Input configuration codes. */
#define CONFIG_PREAMBLE_1     ((uint8_t)0x55U)
#define CONFIG_PREAMBLE_2     ((uint8_t)0x55U)

/* Output acknowledge codes. */
#define CONFIG_ACK_1          ((uint8_t)0x55U)
#define CONFIG_ACK_2          ((uint8_t)0x55U)

/* Input digital output codes. */
#define as_PREAMBLE_1       ((uint8_t)0x4FU)
#define as_PREAMBLE_2      ((uint8_t)0x4FU)

#define DOUT_PREAMBLE_1       ((uint8_t)0xAAU)
#define DOUT_PREAMBLE_2       ((uint8_t)0xAAU)

/* PORTB. */
#define UART3_RX_PIN_NUM                  10U
#define UART3_TX_PIN_NUM                  11U

/* Status flags. */
#define UART3_TDRE_FLAG (((UART3->S1) & UART_S1_TDRE_MASK) >> UART_S1_TDRE_SHIFT)
#define UART3_RDRF_FLAG (((UART3->S1) & UART_S1_RDRF_MASK) >> UART_S1_RDRF_SHIFT)

/* The following defines/macros are used to parse incoming frame data into
 * relevant parts based off of predefined frame structure. */
#define AG_SR_MASK            ((uint8_t)0x03U)
#define AG_SR_SHIFT              ((uint8_t)0U)
#define AG_SR_BYTE_INDEX                   0U
#define FIND_AG_SR(buf)   ((uint8_t)((buf[AG_SR_BYTE_INDEX] & AG_SR_MASK) >> AG_SR_SHIFT))

#define AIN_POW_MASK          ((uint8_t)0x0FU)
#define AIN_POW_SHIFT            ((uint8_t)0U)
#define AIN_POW_BYTE_INDEX                 1U
#define FIND_AIN_POW(buf) ((uint8_t)((buf[AIN_POW_BYTE_INDEX] & AIN_POW_MASK) >> AIN_POW_SHIFT))

#define AIN_SR_MASK           ((uint8_t)0xFFU)
#define AIN_SR_SHIFT             ((uint8_t)0U)
#define AIN_SR_BYTE_INDEX                  2U
#define FIND_AIN_SR(buf) ((uint8_t)((buf[AIN_SR_BYTE_INDEX] & AIN_SR_MASK) >> AIN_SR_SHIFT))

#define DOUT_POW_MASK         ((uint8_t)0xFFU)
#define DOUT_POW_SHIFT           ((uint8_t)0U)
#define DOUT_POW_BYTE_INDEX                0U
#define FIND_DOUT_POW(buf) ((uint8_t)((buf[DOUT_POW_BYTE_INDEX] & DOUT_POW_MASK) >> DOUT_POW_SHIFT))

#define DOUT_ST_MASK          ((uint8_t)0xFFU)
#define DOUT_ST_SHIFT            ((uint8_t)0U)
#define DOUT_ST_BYTE_INDEX                 1U
#define FIND_DOUT_ST(buf) ((uint8_t)((buf[DOUT_ST_BYTE_INDEX] & DOUT_ST_MASK) >> DOUT_ST_SHIFT))

#define DAQ1_POW_MASK         ((uint8_t)0x0FU)
#define DAQ1_POW_SHIFT           ((uint8_t)0U)
#define DAQ1_POW_BYTE_INDEX                3U
#define FIND_DAQ1_POW(buf) ((uint8_t)((buf[DAQ1_POW_BYTE_INDEX] & DAQ1_POW_MASK) >> DAQ1_POW_SHIFT))

#define DAQ2_POW_MASK         ((uint8_t)0xF0U)
#define DAQ2_POW_SHIFT           ((uint8_t)4U)
#define DAQ2_POW_BYTE_INDEX                3U
#define FIND_DAQ2_POW(buf) ((uint8_t)((buf[DAQ2_POW_BYTE_INDEX] & DAQ2_POW_MASK) >> DAQ2_POW_SHIFT))

#define DAQ3_POW_MASK         ((uint8_t)0x0FU)
#define DAQ3_POW_SHIFT           ((uint8_t)0U)
#define DAQ3_POW_BYTE_INDEX                4U
#define FIND_DAQ3_POW(buf) ((uint8_t)((buf[DAQ3_POW_BYTE_INDEX] & DAQ3_POW_MASK) >> DAQ3_POW_SHIFT))

#define DAQ4_POW_MASK         ((uint8_t)0xF0U)
#define DAQ4_POW_SHIFT           ((uint8_t)4U)
#define DAQ4_POW_BYTE_INDEX                4U
#define FIND_DAQ4_POW(buf) ((uint8_t)((buf[DAQ4_POW_BYTE_INDEX] & DAQ4_POW_MASK) >> DAQ4_POW_SHIFT))

#define DAQ5_POW_MASK         ((uint8_t)0x0FU)
#define DAQ5_POW_SHIFT           ((uint8_t)0U)
#define DAQ5_POW_BYTE_INDEX                5U
#define FIND_DAQ5_POW(buf) ((uint8_t)((buf[DAQ5_POW_BYTE_INDEX] & DAQ5_POW_MASK) >> DAQ5_POW_SHIFT))

#define DAQ6_POW_MASK         ((uint8_t)0xF0U)
#define DAQ6_POW_SHIFT           ((uint8_t)4U)
#define DAQ6_POW_BYTE_INDEX                5U
#define FIND_DAQ6_POW(buf) ((uint8_t)((buf[DAQ6_POW_BYTE_INDEX] & DAQ6_POW_MASK) >> DAQ6_POW_SHIFT))

#define DAQ7_POW_MASK         ((uint8_t)0x0FU)
#define DAQ7_POW_SHIFT           ((uint8_t)0U)
#define DAQ7_POW_BYTE_INDEX                6U
#define FIND_DAQ7_POW(buf) ((uint8_t)((buf[DAQ7_POW_BYTE_INDEX] & DAQ7_POW_MASK) >> DAQ7_POW_SHIFT))

#define DAQ8_POW_MASK         ((uint8_t)0xF0U)
#define DAQ8_POW_SHIFT           ((uint8_t)4U)
#define DAQ8_POW_BYTE_INDEX                6U
#define FIND_DAQ8_POW(buf) ((uint8_t)((buf[DAQ8_POW_BYTE_INDEX] & DAQ8_POW_MASK) >> DAQ8_POW_SHIFT))

#define DAQ1_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ1_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ1_TYPE_BYTE_INDEX                7U
#define FIND_DAQ1_TYPE(buf) ((uint8_t)((buf[DAQ1_TYPE_BYTE_INDEX] & DAQ1_TYPE_MASK) >> DAQ1_TYPE_SHIFT))

#define DAQ2_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ2_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ2_TYPE_BYTE_INDEX                8U
#define FIND_DAQ2_TYPE(buf) ((uint8_t)((buf[DAQ2_TYPE_BYTE_INDEX] & DAQ2_TYPE_MASK) >> DAQ2_TYPE_SHIFT))

#define DAQ3_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ3_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ3_TYPE_BYTE_INDEX                9U
#define FIND_DAQ3_TYPE(buf) ((uint8_t)((buf[DAQ3_TYPE_BYTE_INDEX] & DAQ3_TYPE_MASK) >> DAQ3_TYPE_SHIFT))

#define DAQ4_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ4_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ4_TYPE_BYTE_INDEX               10U
#define FIND_DAQ4_TYPE(buf) ((uint8_t)((buf[DAQ4_TYPE_BYTE_INDEX] & DAQ4_TYPE_MASK) >> DAQ4_TYPE_SHIFT))

#define DAQ5_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ5_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ5_TYPE_BYTE_INDEX               11U
#define FIND_DAQ5_TYPE(buf) ((uint8_t)((buf[DAQ5_TYPE_BYTE_INDEX] & DAQ5_TYPE_MASK) >> DAQ5_TYPE_SHIFT))

#define DAQ6_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ6_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ6_TYPE_BYTE_INDEX               12U
#define FIND_DAQ6_TYPE(buf) ((uint8_t)((buf[DAQ6_TYPE_BYTE_INDEX] & DAQ6_TYPE_MASK) >> DAQ6_TYPE_SHIFT))

#define DAQ7_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ7_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ7_TYPE_BYTE_INDEX               13U
#define FIND_DAQ7_TYPE(buf) ((uint8_t)((buf[DAQ7_TYPE_BYTE_INDEX] & DAQ7_TYPE_MASK) >> DAQ7_TYPE_SHIFT))

#define DAQ8_TYPE_MASK         ((uint8_t)0xFFU)
#define DAQ8_TYPE_SHIFT           ((uint8_t)0U)
#define DAQ8_TYPE_BYTE_INDEX               14U
#define FIND_DAQ8_TYPE(buf) ((uint8_t)((buf[DAQ8_TYPE_BYTE_INDEX] & DAQ8_TYPE_MASK) >> DAQ8_TYPE_SHIFT))

typedef enum
{
    VERIFY_PREAMBLE,
    SAVE_DATA,
    VERIFY_CHECKSUM,
    SET_MODULES,
    SEND_ACK,
    ERROR
} user_input_t;

typedef struct tel_tx_state_t
{
    uint8_t ag_tel_state;
    uint8_t ain_tel_state;
} tel_tx_state_t;

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Task handle for Telemetry input task, used by UAR3_RX_TX_IRQHandler() to post
 * task notification. */
static TaskHandle_t telInputTaskHandle = NULL;

/* Task handle for Telemetry output task, used by UAR3_RX_TX_IRQHandler() to post
 * task notification. */
static TaskHandle_t telOutputTaskHandle = NULL;

/* Binary semaphore that will be pended on by the Telemetry output task. Once
 * the input receives and acknowledges initial configuration packet, this
 * semaphore is posted and the output task can proceed to send data packets. */
static SemaphoreHandle_t telInitConfigRecieved;

/* Copies of module data made in preparation for data frame transmission. */
static ag_data_t telAGData;
static gps_data_t telGPSData;
static ain_data_t telAInData;
static daq_data_t telDAQData[8];

/* Buffers to hold the data (not preamble and not checksum) sections of frame. */
static uint8_t telConfigMsgBuffer[CONFIG_FRAME_DATA_BYTES];
static uint8_t telDoutMsgBuffer[DOUT_FRAME_DATA_BYTES];

static volatile uint8_t telTxByteToSend;
static volatile uint8_t telRxByteReceived;

static uint16_t telDataChecksum;

static uint8_t telIsInitConfigRecieved = FALSE;

/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
static void telInputTask(void *pvParameters);

static void telOutputTask(void *pvParameters);

static void telGetDoutState(void);

static void telGetSensorConfig(void);

static void telIdleTaskUntilInterrupt(void);

static void telSendTime(void);

static void telSendAG(void);

static void telSendAIn(void);

static void telSendDAQ(void);

static void telSetAIn(uint8_t *);

static void telSetAG(uint8_t *);

static void telSetDOut(uint8_t *);

static void telSetDAQ(uint8_t *);

static void telResurrectModule(void);

/******************************************************************************
*   TelInit() - Public function to configure UART3 for reception and
*   transmission with the separate wireless telemetry unit.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
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

    telInitConfigRecieved = xSemaphoreCreateBinary();
    while(telInitConfigRecieved == NULL){ /* Out of heap memory (DEBUG TRAP). */ }

    task_create_return = xTaskCreate(telInputTask,
                                     "Telemetry Input Task",
                                     TELINPUTTASK_STKSIZE,
                                     NULL,
                                     TELINPUTTASK_PRIORITY,
                                     &telInputTaskHandle);

    while(task_create_return == pdFAIL){ /* Out of heap memory (DEBUG TRAP). */ }

    task_create_return = xTaskCreate(telOutputTask,
                                     "Telemetry Output Task",
                                     TELOUTPUTTASK_STKSIZE,
                                     NULL,
                                     TELOUTPUTTASK_PRIORITY,
                                     &telOutputTaskHandle);

    while(task_create_return == pdFAIL){  /* Out of heap memory (DEBUG TRAP). */ }

    NVIC_SetPriority(UART3_RX_TX_IRQn, 2U);
    NVIC_ClearPendingIRQ(UART3_RX_TX_IRQn);
    NVIC_EnableIRQ(UART3_RX_TX_IRQn);
}

/******************************************************************************
*   telInputTask() - This FreeRTOS task consists of sequential operation of
*   saving all user-input configuration data transmitted from wireless telemetry
*   and digital output states via UART3. A non-blocking, FreeRTOS friendly UART
*   driver will be used to pend on a new byte reception. While this should be
*   occurring at 2Hz, exact timing is dependent on telemetry unit.
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
static void telInputTask(void *pvParameters)
{
    UART3->C2 |= (UART_C2_RE(ENABLE) | UART_C2_RIE(ENABLE));

    while(1)
    {
        telIdleTaskUntilInterrupt();
        switch(telRxByteReceived)
        {
            /* Incoming configuration frame. */
            case CONFIG_PREAMBLE_1:
                telGetSensorConfig();
                break;

            /* Incoming digital output state frame. */
            case DOUT_PREAMBLE_1:
            case as_PREAMBLE_1:
                telGetDoutState();
                break;

            default:
                break;
        }
    }
}

/******************************************************************************
*   telOutputTask() - This FreeRTOS task consists of sequential operation of
*   grabbing the current data in all modules, then transmitting the data over
*   UART3 to the wireless telemetry unit with a non-blocking, FreeRTOS friendly
*   UART driver. This task repeats operation at 2Hz. Only starts transmission
*   once initial configuration frame has been received and acknowledged.
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
static void telOutputTask(void *pvParameters)
{
    uint16_t checksum_to_send;
    xSemaphoreTake(telInitConfigRecieved, portMAX_DELAY);

    NVIC_ClearPendingIRQ(UART3_RX_TX_IRQn);
    UART3->C2 |= UART_C2_TE(ENABLE);
    telTxByteToSend = CONFIG_ACK_1;
    UART3->D = telTxByteToSend;
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = CONFIG_ACK_1;
    telIdleTaskUntilInterrupt();
    telTxByteToSend = CONFIG_ACK_2;
    telIdleTaskUntilInterrupt();
    UART3->C2 &= ~UART_C2_TIE_MASK;

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(500U));

        /* Grabbing all data. */
        GPSGetData(&telGPSData);
        AGGetData(&telAGData);
        AInGetData(&telAInData);
        DAQGetData(telDAQData);


        telTxByteToSend = 0xAA;
        UART3->C2 |= UART_C2_TIE(ENABLE);
        telIdleTaskUntilInterrupt();
        telTxByteToSend = 0xAA;
        telIdleTaskUntilInterrupt();
        UART3->C2 &= ~UART_C2_TIE_MASK;

        telDataChecksum = 0;

        /* Sending data. */
        telSendTime();
        telSendAG();
        telSendAIn();

        telTxByteToSend = 0;
        UART3->C2 |= UART_C2_TIE(ENABLE);
        telIdleTaskUntilInterrupt();
        for(uint8_t i = 1; i < 32; i++)
        {
            telTxByteToSend = i;
            telIdleTaskUntilInterrupt();
        }
        UART3->C2 &= ~UART_C2_TIE_MASK;

        telSendDAQ();

        checksum_to_send = telDataChecksum;

        telTxByteToSend = ((uint8_t)(checksum_to_send >> 8));
        UART3->C2 |= UART_C2_TIE(ENABLE);
        telIdleTaskUntilInterrupt();
        telTxByteToSend = (uint8_t)checksum_to_send;
        telIdleTaskUntilInterrupt();
        UART3->C2 &= ~UART_C2_TIE_MASK;
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
*
*   Author: Anthony Needles
******************************************************************************/
void UART3_RX_TX_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    if(UART3_TDRE_FLAG == SET)
    { /* Sending data, Tx buffer is empty. */
        telDataChecksum += telTxByteToSend;
        UART3->D = telTxByteToSend;
        vTaskNotifyGiveFromISR(telOutputTaskHandle, &xHigherPriorityTaskWoken);
    } else {}

    if(UART3_RDRF_FLAG == SET)
    { /* Reading data, Rx buffer is full. */
        telRxByteReceived = UART3->D;
        vTaskNotifyGiveFromISR(telInputTaskHandle, &xHigherPriorityTaskWoken);
    } else {}

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   telGetSensorConfig() - Non-blocking, FreeRTOS friendly UART driver for
*   receiving and handling incoming configuration frame. Unlocks output task
*   when initial configuration frame is received and acknowledged. May occur
*   at any time (whenever user requests reconfiguration).
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telGetSensorConfig()
{
    uint8_t rx_status = ENABLED;
    uint16_t rx_checksum;
    uint16_t calc_checksum;
    user_input_t state = VERIFY_PREAMBLE;

    while(rx_status == ENABLED)
    {
        switch(state)
        {
            case VERIFY_PREAMBLE:
                /* Ensure that second byte of preamble is received. */
                telIdleTaskUntilInterrupt();
                if(telRxByteReceived != CONFIG_PREAMBLE_2)
                {
                    rx_status = DISABLED;
                    state = VERIFY_PREAMBLE;
                } else
                {
                    state = SAVE_DATA;
                }

                break;

            case SAVE_DATA:
                /* Capture all data section of frame. This does not include
                 * preamble nor checksum. */
                for(uint8_t i = 0; i < CONFIG_FRAME_DATA_BYTES; i++)
                {
                    telIdleTaskUntilInterrupt();
                    telConfigMsgBuffer[i] = telRxByteReceived;
                }

                state = VERIFY_CHECKSUM;
                break;

            case VERIFY_CHECKSUM:
                rx_checksum = 0;
                /* Capture 16-bit checksum of data. */
                telIdleTaskUntilInterrupt();
                rx_checksum |= (((uint16_t)telRxByteReceived) << 8U);

                telIdleTaskUntilInterrupt();
                rx_checksum |= ((uint16_t)telRxByteReceived);


                /* Perform summation of data section of frame. */
                calc_checksum = 0;
                for(uint16_t i = 0; i < CONFIG_FRAME_DATA_BYTES; i++)
                {
                    calc_checksum += telConfigMsgBuffer[i];
                }

//                if(calc_checksum != rx_checksum)
//                {
//                    rx_status = DISABLED;
//                    state = VERIFY_PREAMBLE;
//                } else
//                {
                    state = SET_MODULES;
//                }
                break;

            case SET_MODULES:
                /* Extract configuration data from frame and set corresponding
                 * modules. */
                telSetAG(telConfigMsgBuffer);
                telSetAIn(telConfigMsgBuffer);
                telSetDAQ(telConfigMsgBuffer);

                if(telIsInitConfigRecieved == FALSE)
                {
                    telIsInitConfigRecieved = TRUE;
                    state = SEND_ACK;
                } else
                {
                    rx_status = DISABLED;
                    state = VERIFY_PREAMBLE;
                }
                break;

            case SEND_ACK:
                /* Enable output task operation. */
                xSemaphoreGive(telInitConfigRecieved);

                rx_status = DISABLED;
                state = VERIFY_PREAMBLE;
                break;

            default:
                rx_status = DISABLED;
                state = VERIFY_PREAMBLE;
                break;
        }
    }
}

/******************************************************************************
*   telGetSensorConfig() - Non-blocking, FreeRTOS friendly UART driver for
*   receiving and handling incoming digital output states. Occurs at 2Hz
*   (expected of wireless telemetry unit).
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telGetDoutState()
{
    uint8_t rx_status = ENABLED;
    user_input_t state = VERIFY_PREAMBLE;

    while(rx_status == ENABLED)
    {
        switch(state)
        {
            case VERIFY_PREAMBLE:
                /* Ensure that second byte of preamble is received. */
                telIdleTaskUntilInterrupt();
                if((telRxByteReceived != DOUT_PREAMBLE_2) && (telRxByteReceived != as_PREAMBLE_1))
                {
                    rx_status = DISABLED;
                    break;
                }

                state = SAVE_DATA;
                break;

            case SAVE_DATA:
                /* Capture all data section of frame. This does not include
                 * preamble. */
                for(uint8_t i = 0; i < DOUT_FRAME_DATA_BYTES; i++)
                {
                    telIdleTaskUntilInterrupt();
                    telDoutMsgBuffer[i] = telRxByteReceived;
                }

                state = SET_MODULES;
                break;

            case SET_MODULES:
                /* Extract digital output state data from frame and set
                 * corresponding module. */
                telSetDOut(telDoutMsgBuffer);

                rx_status = DISABLED;
                state = VERIFY_PREAMBLE;
                break;

            default:
                rx_status = DISABLED;
                break;
        }
    }
}

/******************************************************************************
*   telIdleTaskUntilInterrupt() - Private function that pends on the UART3 ISR. This
*   will both when the UART3 TX buffer is empty and when the UART3 RX buffer
*   is full. The ISR will check which flag triggered the interrupt, and notifies
*   the corresponding task. This ISR allows the telemetry tasks to enter an
*   idle state and allow the CPU to active other tasks.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telIdleTaskUntilInterrupt()
{
    uint8_t notify_count;

    /* Place task into idle state until UART3 ISR notifies task. */
    notify_count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if(notify_count == 0)
    { /* Resurrect if failed to be notified. */
        telResurrectModule();
    } else {}
}

/******************************************************************************
*   telSendTime() - Private function that transmits captured GPS data by setting
*   global variable that will be written to UART3 data register upon transmit
*   buffer full interrupt. Non-blocking and FreeRTOS friendly.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSendTime()
{
    telTxByteToSend = telGPSData.year;
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = telGPSData.month;
    telIdleTaskUntilInterrupt();
    telTxByteToSend = telGPSData.day;
    telIdleTaskUntilInterrupt();
    telTxByteToSend = telGPSData.hour;
    telIdleTaskUntilInterrupt();
    telTxByteToSend = telGPSData.min;
    telIdleTaskUntilInterrupt();
    telTxByteToSend = telGPSData.sec;
    telIdleTaskUntilInterrupt();
    telTxByteToSend = telGPSData.cs;
    telIdleTaskUntilInterrupt();
    UART3->C2 &= ~UART_C2_TIE_MASK;
}

/******************************************************************************
*   telSendAG() - Private function that transmits captured Accel/Gyro data by
*   setting global variable that will be written to UART3 data register upon
*   transmit buffer full interrupt. 16 bit values are sent upper byte first,
*   then lower byte. Non-blocking and FreeRTOS friendly.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSendAG()
{
    telTxByteToSend = ((uint8_t)(telAGData.accel_data.x >> 8));
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.accel_data.x);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.accel_data.y >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.accel_data.y);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.accel_data.z >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.accel_data.z);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.gyro_data.x >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.gyro_data.x);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.gyro_data.y >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.gyro_data.y);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAGData.gyro_data.z >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAGData.gyro_data.z);
    telIdleTaskUntilInterrupt();
    UART3->C2 &= ~UART_C2_TIE_MASK;
}

/******************************************************************************
*   telSendAG() - Private function that transmits captured Accel/Gyro data by
*   setting global variable that will be written to UART3 data register upon
*   transmit buffer full interrupt. 16 bit values are sent upper byte first,
*   then lower byte. Non-blocking and FreeRTOS friendly.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSendAIn()
{
    telTxByteToSend = ((uint8_t)(telAInData.ain1_data >> 8));
    UART3->C2 |= UART_C2_TIE(ENABLE);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain1_data);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAInData.ain2_data >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain2_data);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAInData.ain3_data >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain3_data);
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)(telAInData.ain4_data >> 8));
    telIdleTaskUntilInterrupt();
    telTxByteToSend = ((uint8_t)telAInData.ain4_data);
    telIdleTaskUntilInterrupt();
    UART3->C2 &= ~UART_C2_TIE_MASK;
}

/******************************************************************************
*   telSendDAQ() -
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSendDAQ()
{
    for(uint8_t i = 0; i < 8; i++)
    {
        for(uint8_t j = 0; j < 4; j++)
        {
            telTxByteToSend = (uint8_t)(telDAQData[i].sensor[j].data >> 8);
            UART3->C2 |= UART_C2_TIE(ENABLE);
            telIdleTaskUntilInterrupt();
            telTxByteToSend = (uint8_t)(telDAQData[i].sensor[j].data);
            telIdleTaskUntilInterrupt();
        }
    }

    UART3->C2 &= ~UART_C2_TIE_MASK;
}

/******************************************************************************
*   telSetAG() - Private function that sets accel/gyro sample rates (usedm by
*   the Accel/Gyro module). This data is received via UART3 by the wireless
*   telemetry unit.
*
*   Parameters:
*
*       uint8_t *msg_buf - Message buffer containing all configuration data
*       from the wireless telemetry unit.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSetAG(uint8_t *msg_buf)
{
    ag_msg_t ag_msg;

    ag_msg.sampling_rate_field = FIND_AG_SR(msg_buf);

    AGSet(ag_msg);
}

/******************************************************************************
*   telSetAIn() - Private function that sets analog input sample rates and power
*   states (used by the Analog Input module). This data is received via UART3 by
*   the wireless telemetry unit.
*
*   Parameters:
*
*       uint8_t *msg_buf - Message buffer containing all configuration data
*       from the wireless telemetry unit.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSetAIn(uint8_t *msg_buf)
{
    ain_msg_t ain_msg;

    ain_msg.sampling_rate_field = FIND_AIN_SR(msg_buf);

    ain_msg.power_state_field = FIND_AIN_POW(msg_buf);

    AInSet(ain_msg);
}

/******************************************************************************
*   telSetDAQ() -
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSetDAQ(uint8_t *msg_buf)
{
    daq_msg_t daq_msg;

    daq_msg.power_fields[DAQ1] = FIND_DAQ1_POW(msg_buf);
    daq_msg.power_fields[DAQ2] = FIND_DAQ2_POW(msg_buf);
    daq_msg.power_fields[DAQ3] = FIND_DAQ3_POW(msg_buf);
    daq_msg.power_fields[DAQ4] = FIND_DAQ4_POW(msg_buf);
    daq_msg.power_fields[DAQ5] = FIND_DAQ5_POW(msg_buf);
    daq_msg.power_fields[DAQ6] = FIND_DAQ6_POW(msg_buf);
    daq_msg.power_fields[DAQ7] = FIND_DAQ7_POW(msg_buf);
    daq_msg.power_fields[DAQ8] = FIND_DAQ8_POW(msg_buf);

    daq_msg.type_fields[DAQ1] = FIND_DAQ1_TYPE(msg_buf);
    daq_msg.type_fields[DAQ2] = FIND_DAQ2_TYPE(msg_buf);
    daq_msg.type_fields[DAQ3] = FIND_DAQ3_TYPE(msg_buf);
    daq_msg.type_fields[DAQ4] = FIND_DAQ4_TYPE(msg_buf);
    daq_msg.type_fields[DAQ5] = FIND_DAQ5_TYPE(msg_buf);
    daq_msg.type_fields[DAQ6] = FIND_DAQ6_TYPE(msg_buf);
    daq_msg.type_fields[DAQ7] = FIND_DAQ7_TYPE(msg_buf);
    daq_msg.type_fields[DAQ8] = FIND_DAQ8_TYPE(msg_buf);

    DAQSet(daq_msg);
}

/******************************************************************************
*   telSetDOut() - Private function that sets digital output powers and states
*   (used by the Digital Output module). This data is received via UART3 by the
*   wireless telemetry unit.
*
*   Parameters:
*
*       uint8_t *msg_buf - Message buffer containing all configuration data
*       from the wireless telemetry unit.
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telSetDOut(uint8_t *msg_buf)
{
    dout_msg_t dout_msg;

    dout_msg.power_field = FIND_DOUT_POW(msg_buf);

    dout_msg.state_field = FIND_DOUT_ST(msg_buf);

    DOutSet(dout_msg);
}

/******************************************************************************
*   telResurrectModule() - Private function that attempts so solve a timeout
*   error by reinitializing module. Disables preemption and interrupts of a
*   system priority <= configMAX_SYSCALL_INTERRUPT_PRIORITY (effectively, these
*   are interrupts set to have a priority <= 1 by NVIC_SetPriority()) when
*   deleting RTOS structures.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
static void telResurrectModule()
{
    taskENTER_CRITICAL();

    NVIC_DisableIRQ(UART3_RX_TX_IRQn);
    vSemaphoreDelete(telInitConfigRecieved);
    vTaskDelete(telInputTaskHandle);
    vTaskDelete(telOutputTaskHandle);

    taskEXIT_CRITICAL();

    TelInit();
}
