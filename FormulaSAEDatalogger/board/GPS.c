/******************************************************************************
*   GPS.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The GPS module of the Formula SAE Datalogger obtains real-world time and
*   date data for accurate timestamps. This configuration will be done once upon
*   program starting initialization to set the time/date starting point.
*   Communication from the GPS unit occurs on UART4 at 9600 baud.
*
*   GPS Unit: L80-R
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/22/2019
*
*   Created on: 05/10/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "GPS.h"
#include "Debug.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                         1U
#define SET                            1U
#define DISABLED                    0x00U
#define ENABLED                     0x01U
#define BAUD_RATE_DIV              0x186U
#define BAUD_RATE_FA                0x0CU
#define SINGLE_STOP_BIT                0U
#define NO_PARITY_BIT                  0U
#define EIGHT_BIT_MODE                 0U
#define SIZE_32_BYTES               0x04U
#define ALT_3_UART                  0x03U
#define PIT0                        0x00U
#define PIT_100HZ_LDVAL           599999U
#define RX_BYTES                      12U
#define DOUT1_STATE_PIN_NUM            1U
#define TIMEOUT_MS                    14U

#define GPSTENMSTASK_STKSIZE         256U
#define GPSTENMSTASK_PRIORITY         7U

/* PORTE. */
#define UART4_RX_PIN_NUM              25U
#define UART4_TX_PIN_NUM              24U

/* Status flags. */
#define RX_DATA_FLAG (((UART4->S1) & UART_S1_RDRF_MASK) >> UART_S1_RDRF_SHIFT)

/* Enumerations for GPS UART Rx state machine */
typedef enum
{
    FIND_PREAMBLE,
    FRAME_DELAY,
    FIND_MSG_ID,
    SAVE_TIME,
    TIME_SET
} time_set_state_t;

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Time and date data received from GPS */
static gps_data_t gpsCurrentData;

/* Mutex key that will protect gps time structure. Must be pended on
 * if writing/reading gpsCurrentData is desired to ensure synchronization. */
static SemaphoreHandle_t gpsCurrentDataKey;


/* Task handle for gps 10ms task, used by PIT0_IRQHandler() to post
 * task notification. */
static TaskHandle_t gpsTenMSTaskHandle = NULL;

/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
static void gpsGetTimeDateBlocking(void);

static void gpsTenMSTask(void *);

/******************************************************************************
*   GPSInit() - Public function to configure UART4 for reception of GPS
*   NMEA time and date data. This data will serve as the starting point of
*   the datalogger's timestamps.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void GPSInit()
{
    BaseType_t task_create_return;

    SIM->SCGC1 |= SIM_SCGC1_UART4(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTE(ENABLE);

    PORTE->PCR[UART4_RX_PIN_NUM] = PORT_PCR_MUX(ALT_3_UART);
    PORTE->PCR[UART4_TX_PIN_NUM] = PORT_PCR_MUX(ALT_3_UART);

    /* Baud rate = Module clock / (16 × (BAUD_RATE_DIV + BAUD_RATE_FA))
     * Module clock = 60MHz, BRD = 390, BRFA = 0.375
     * Baud rate = ~9606 */
    UART4->BDH = (uint8_t)(BAUD_RATE_DIV >> 8);
    UART4->BDL = (uint8_t)(BAUD_RATE_DIV);
    UART4->C4 = (uint8_t)(BAUD_RATE_FA);

    UART4->C1 =  (UART_C1_PE(NO_PARITY_BIT) | UART_C1_M(EIGHT_BIT_MODE));

    /* PIT0 initialization for 10ms period, 100Hz trigger frequency. */
    SIM->SCGC6 |= SIM_SCGC6_PIT(ENABLE);
    PIT->MCR &= ~PIT_MCR_MDIS(ENABLE);
    PIT->CHANNEL[PIT0].LDVAL = PIT_100HZ_LDVAL;
    PIT->CHANNEL[PIT0].TCTRL |= (PIT_TCTRL_TIE(ENABLE) | PIT_TCTRL_TEN(ENABLE));

    gpsCurrentData.year  = 0x00U;
    gpsCurrentData.month = 0x00U;
    gpsCurrentData.day   = 0x00U;
    gpsCurrentData.hour  = 0x00U;
    gpsCurrentData.min   = 0x00U;
    gpsCurrentData.sec   = 0x00U;
    gpsCurrentData.ms    = 0x00U;

    gpsCurrentDataKey = xSemaphoreCreateMutex();
    while(gpsCurrentDataKey == NULL){ /* Error trap */ }

    task_create_return = xTaskCreate(gpsTenMSTask,
                                     "GPS Ten MS Task",
                                     GPSTENMSTASK_STKSIZE,
                                     NULL,
                                     GPSTENMSTASK_PRIORITY,
                                     &gpsTenMSTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    //    gpsGetTimeDateBlocking();

    NVIC_SetPriority(PIT0_IRQn, 2U);
    NVIC_ClearPendingIRQ(PIT0_IRQn);
    NVIC_EnableIRQ(PIT0_IRQn);
}

/******************************************************************************
*   gpsTenMSTask() - Upon task post by PIT0 IRQ (every 10ms), increments
*   tens-of-milliseconds count, handling all overflow for seconds, minutes,
*   hours, days, months, years. Handles differing day count in months, except
*   for leap years. Does not compute day light savings.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
******************************************************************************/
static void gpsTenMSTask(void *pvParameters)
{
    uint8_t notify_count;

    while(1)
    {
        /* Place task into idle state until PIT0 ISR notifies task. */
        notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(TIMEOUT_MS));
        if(notify_count == 0)
        { /* If this takes longer than 1ms, go to ERROR state. */
            while(1){}
        } else {}

        xSemaphoreTake(gpsCurrentDataKey, portMAX_DELAY);

        /*
                                                                                  .dm+
                                                                                  -NMs
         :hs``/shhy+-                ./syhhy+.               -+yhhys+.          ohdMMmhhhs.
         +MNhNdsosdMMy`            .yMmy+//sdMd-           `yMNo///oy-          :+sNMh+++/`
         +MMh:     oMM+           .mMh.     `hMm.          -NMo                   -NMs
         +MN/      :NMo           +MMhooooooohMN/           oNMds/.`              -NMs
         +MN/      :NMs           sMMhssssssssso.            `:ohmNmy-            -NMs
         +MN/      :NMs           :MMo                            -dMm.           -NMs
         +MN/      :NMs            sMMy:.```-/s+           /o:.```/dMh.           .mMN:..-`
         +MN:      -NMo             -sdNNNNNmdy-           :hmNNNNmdo.             :hNMMNh.
         `..        ..`                ``..``                ``...`                  `..`

        */
        if(gpsCurrentData.ms >= 99U){
            gpsCurrentData.ms = 0;
            if(gpsCurrentData.sec >= 59U){
                gpsCurrentData.sec = 0;
                if(gpsCurrentData.min >= 59U){
                    gpsCurrentData.min = 0;
                    if(gpsCurrentData.hour >= 23U){
                        gpsCurrentData.hour = 0;
                        /* thank pope greg */
                        switch(gpsCurrentData.month)
                        {
                            case 2:
                                if(gpsCurrentData.day >= 28U){
                                    gpsCurrentData.day = 1;
                                    if(gpsCurrentData.month >= 12U){
                                        gpsCurrentData.month = 1;
                                        gpsCurrentData.year++;
                                    } else {
                                        gpsCurrentData.month++;
                                    }
                                } else {
                                    gpsCurrentData.day++;
                                }
                                break;

                            case 4:
                            case 6:
                            case 9:
                            case 11:
                                if(gpsCurrentData.day >= 30U){
                                    gpsCurrentData.day = 1;
                                    if(gpsCurrentData.month >= 12U){
                                        gpsCurrentData.month = 1;
                                        gpsCurrentData.year++;
                                    } else {
                                        gpsCurrentData.month++;
                                    }
                                } else {
                                    gpsCurrentData.day++;
                                }
                                break;

                            case 1:
                            case 3:
                            case 5:
                            case 7:
                            case 8:
                            case 10:
                            case 12:
                                if(gpsCurrentData.day >= 31U){
                                    gpsCurrentData.day = 1;
                                    if(gpsCurrentData.month >= 12U){
                                        gpsCurrentData.month = 1;
                                        gpsCurrentData.year++;
                                    } else {
                                        gpsCurrentData.month++;
                                    }
                                } else {
                                    gpsCurrentData.day++;
                                }
                                break;

                            default:
                                break;
                        }
                    } else {
                        gpsCurrentData.hour++;
                    }
                } else {
                    gpsCurrentData.min++;
                }
            } else {
                gpsCurrentData.sec++;
            }
        } else {
            gpsCurrentData.ms++;
        }

        xSemaphoreGive(gpsCurrentDataKey);
    }
}

/******************************************************************************
*   PIT0_IRQHandler() - Interrupt handler for PIT0 TIF (timer interrupt) flag.
*   Occurs at 100Hz. Triggers 10ms increment by posting task notify to
*   gpsTenMSTask.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void PIT0_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    PIT->CHANNEL[PIT0].TFLG |= PIT_TFLG_TIF_MASK;

    vTaskNotifyGiveFromISR(gpsTenMSTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   GPSGetData() - Public function to copy current GPS data structure
*   for use in transmission/storage.
*
*   Parameters:
*
*       gps_time_date_data_t *ldata - Pointer to caller-side data structure
*       which will have current data copied to it.
*
*   Return: None
******************************************************************************/
void GPSGetData(gps_data_t *ldata)
{
    /* Pend on Mutex to update data structure */
    xSemaphoreTake(gpsCurrentDataKey, portMAX_DELAY);

    *ldata = gpsCurrentData;

    xSemaphoreGive(gpsCurrentDataKey);
}

/******************************************************************************
*   gpsGetTimeDateBlocking() - Private function that features a state machine
*   to read the NMEA 0183 time and date data being received from the GPS. The
*   desired NMEA sentence is RMC (position, velocity, and time), which is the
*   first section of the 6-sentence-wide frame that the GPS transmits at 1Hz.
*   Polling is used to detect when a byte is received, thus blocking the
*   program (this is only ran during initialization, so this is okay).
*
*   FIND_PREAMBLE - Reads UART Rx to find a '$', the preamble character in the
*   RMC NMEA message. Most of the time, this UART line will be idling in between
*   1Hz transmissions, so this Rx read will most likely receive the preamble of
*   the first section of the NMEA frame (the RMC sentence). However, it is
*   possible that this Rx read will occur in the middle of a frame, in which
*   case the opportunity to receive the first sentence is lost until the next
*   frame.
*
*   FIND_MSG_ID - After finding the section preamble, this state will verify
*   that the current section is the RMC sentence, rather than any of the other
*   five that comprise the whole NMEA frame. The talker and message ID for the
*   RMC sentence is "GPRMC".
*
*   SAVE_TIME - Extracts time and date data from the following transmitted
*   characters. The next characters is a ','. followed by 6 Rx characters that
*   comprise time data in hhmmss format, followed by 5 unnecessary characters,
*   followed by the time valid character ('A' = valid, 'V' = invalid), followed
*   by 37 unnecessary characters, followed by a ',', and finally 6 Rx characters
*   that comprise date data in DDMMYY. If an unexpected character is found, or
*   a 'V' (invalid) time is detected, the state machine starts over.
*
*   TIME_SET - Converts received ASCII characters to a decimal value, and saves
*   them in a structure.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void gpsGetTimeDateBlocking()
{
    uint8_t rx_status = ENABLED;
    time_set_state_t state = FIND_PREAMBLE;
    uint8_t temp_read = 0;
    uint8_t invalid_flag;
    uint8_t gpsMessage[RX_BYTES];

    /* Get rid of warning. */
    temp_read++;

    while(rx_status == ENABLED)
    {
        switch(state)
        {
            case FIND_PREAMBLE:
                invalid_flag = 0;
                UART4->C2 |= UART_C2_RE(ENABLE);
                while(RX_DATA_FLAG != SET){}
                state = ((UART4->D == '$') ? FIND_MSG_ID : FIND_PREAMBLE);
                break;

            case FIND_MSG_ID:
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'G'){ state = FIND_PREAMBLE; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'P'){ state = FIND_PREAMBLE; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'R'){ state = FIND_PREAMBLE; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'M'){ state = FIND_PREAMBLE; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'C'){ state = FIND_PREAMBLE; }

                state = SAVE_TIME;
                break;

            case SAVE_TIME:
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != ','){ invalid_flag = 1U; }

                for(int i = 0; i < 6; i++)
                {
                    while(RX_DATA_FLAG != SET){}
                    gpsMessage[i] = UART4->D;
                }

                for(int i = 0; i < 5; i++)
                {
                    while(RX_DATA_FLAG != SET){}
                    temp_read = UART4->D;
                }

                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'A'){ invalid_flag = 1U; }

                for(int i = 0; i < 37; i++)
                {
                    while(RX_DATA_FLAG != SET){}
                    temp_read = UART4->D;
                }

                while(RX_DATA_FLAG != SET){}
                if(UART4->D != ','){ invalid_flag = 1U; }

                for(int i = 6; i < 12; i++)
                {
                    while(RX_DATA_FLAG != SET){}
                    gpsMessage[i] = UART4->D;
                }

                (invalid_flag == SET) ? state = FIND_PREAMBLE : TIME_SET;
                break;

            case TIME_SET:
                UART4->C2 &= ~UART_C2_RE(ENABLE);

                gpsCurrentData.day   = (gpsMessage[6] - '0')*10U +
                                        (gpsMessage[7] - '0');

                gpsCurrentData.year  = (gpsMessage[10] - '0')*10U +
                                        (gpsMessage[11] - '0');

                gpsCurrentData.month = (gpsMessage[8] - '0')*10U +
                                        (gpsMessage[9] - '0');

                gpsCurrentData.hour  = (gpsMessage[0] - '0')*10U +
                                        (gpsMessage[1] - '0');

                gpsCurrentData.min   = (gpsMessage[2] - '0')*10U +
                                        (gpsMessage[3] - '0');

                gpsCurrentData.sec   = (gpsMessage[4] - '0')*10U +
                                        (gpsMessage[5] - '0');

                rx_status = DISABLED;
                break;

            default:
                break;
        }
    }
}
