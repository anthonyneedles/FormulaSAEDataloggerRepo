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
*   Communication from the GPS unit occurs on UART4 at 1Hz.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/12/2019
*
*   Created on: 05/10/2019
*   Author: Anthony Needles
******************************************************************************/
#include "GPS.h"
#include "MK66F18.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
//void gpsGetTimeTask(void *);

void gpsGetTimeDateBlocking(void);

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                      1U
#define SET                         1U
#define DISABLED                 0x01U
#define ENABLED                  0x01U
#define BAUD_RATE_DIV           0x186U
#define BAUD_RATE_FA             0x18U
#define SINGLE_STOP_BIT             0U
#define NO_PARITY_BIT               0U
#define EIGHT_BIT_MODE              0U
#define SIZE_32_BYTES            0x04U
#define GPSTIMECOMPTASK_STKSIZE   256U
#define GPSTIMECOMPTASK_PRIORITY    5U
#define GPSTIMECOMPTASK_WAIT_MS 60000U

#define UART4_RX_PIN_NUM           25U
#define UART4_TX_PIN_NUM           24U
#define ALT_3_UART               0x03U
#define RX_BYTES                   17U

#define RX_DATA_FLAG (((UART4->S1) & UART_S1_RDRF_MASK) >> UART_S1_RDRF_SHIFT)

/* Enumerations for GPS UART Rx state machine */
typedef enum
{
    FIND_PREAMBLE,
    FRAME_DELAY,
    FIND_MSG_ID,
    SAVE_TIME,
    TIME_SET
} TimeCompState_t;

/* Time and date structures held as decimal values (yymmdd hhmmss.uuu) */
typedef struct gpsTimeDateData_t
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t ms;
} gpsTimeDateData_t;

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Task handle for GPS time compensation task, used by UART4_RX_TX_IRQHandler to
 * post task notification */
//static TaskHandle_t gpsTimeCompTaskHandle = NULL;

/* Time and date data received from GPS */
static gpsTimeDateData_t gpsTimeDateData;


/******************************************************************************
*   GPSUART4Init() - Public function to configure UART4 for reception of GPS
*   NMEA time and date data. This data will serve as the starting point of
*   the datalogger's timestamps.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void GPSUART4Init()
{
//    BaseType_t task_create_return;

    SIM->SCGC1 |= SIM_SCGC1_UART4(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTE(ENABLE);

    PORTB->PCR[UART4_RX_PIN_NUM] = PORT_PCR_MUX(ALT_3_UART);
    PORTB->PCR[UART4_TX_PIN_NUM] = PORT_PCR_MUX(ALT_3_UART);

    /* Baud rate = Module clock / (16 × (BAUD_RATE_DIV + BAUD_RATE_FA))
     * Module clock = 60MHz, BRD = 390, BRFA = 0.375
     * Baud rate = ~9606 */
    UART4->BDH = ((UART4->BDH & ~UART_BDH_SBR_MASK &
                   ~UART_BDH_SBNS_MASK) |
                   UART_BDH_SBNS(SINGLE_STOP_BIT) |
                   (uint8_t)(BAUD_RATE_DIV >> 8));

    UART4->BDL = (uint8_t)(BAUD_RATE_DIV);
    UART4->C4 = (uint8_t)(BAUD_RATE_FA);

    /* No parity bit, eight bit mode */
    UART4->C1 = ((UART4->C1 & ~UART_C1_PE_MASK & ~UART_C1_M_MASK) |
                  UART_C1_PE(NO_PARITY_BIT) |
                  UART_C1_M(EIGHT_BIT_MODE));

    gpsGetTimeDateBlocking();

    /* Enable UART4 interrupts (for RIE ISR), with priority change for use
     * with FreeRTOS */
//    NVIC_SetPriority(UART4_RX_TX_IRQn, 2U);
//    NVIC_ClearPendingIRQ(UART4_RX_TX_IRQn);
//    NVIC_EnableIRQ(UART4_RX_TX_IRQn);

    /* Create GPS time compensation task */
//    task_create_return = xTaskCreate(gpsGetTimeTask,
//                                     "GPS Time Compensation Task",
//                                     GPSTIMECOMPTASK_STKSIZE,
//                                     NULL,
//                                     GPSTIMECOMPTASK_PRIORITY,
//                                     &gpsTimeCompTaskHandle);
//
//    while(task_create_return == pdFAIL){ /* Error trap */ }
}

/******************************************************************************
*   gpsGetTimeDateBlocking() - Private function that features a state machine
*   to read the NMEA 0183 time and date data being received from the GPS. The
*   desired NMEA sentence is RMC (position, velocity, and time), which is the
*   first section of the 6-sentence-wide frame that the GPS transmits at 1Hz.
*   Polling is used to detect when a byte is received, thus blocking the
*   program.
*
*   FIND_PREAMBLE - Reads UART Rx to find a '$', the preamble character in the
*   RMC NMEA message. Most of the time, this UART line will be idling in between
*   1Hz transmissions, so this Rx read will most likely receive the preamble of
*   the first section of the NMEA frame (the RMC sentence). However, it is
*   possible that this Rx read will occur in the middle of a frame, in which
*   case the opportunity to receive the first sentence is lost until the next
*   frame.
*
*   FRAME_DELAY - Delays the state machine by ~500ms, which is guaranteed to
*   skip the entire current frame and place the next Rx read at the beginning of
*   the next frame. This guarantees that the next read character will be the RMC
*   preamble. Transitions to this state occur whenever the current Rx data is
*   invalid (time is invalid, wrong section, no preamble found, unexpected
*   character, etc.).
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
*   that comprise date data in DDMMYY.
*
*   TIME_SET - Converts received ASCII characters to a decimal value, and saves
*   them in a structure.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void gpsGetTimeDateBlocking()
{
    uint8_t rx_status = ENABLED;
    TimeCompState_t state = FIND_PREAMBLE;
    uint8_t temp_read;
    uint8_t gpsMessage[RX_BYTES];

    while(rx_status == ENABLED)
    {
        switch(state)
        {
            case FIND_PREAMBLE:
                UART4->C2 |= UART_C2_RE(ENABLE);
                while(RX_DATA_FLAG != SET){}
                state = ((UART4->D == '$') ? FIND_MSG_ID : FRAME_DELAY);
                break;

            case FRAME_DELAY:
                UART4->C2 &= ~UART_C2_RE(ENABLE);
                for(int i = 0; i < 90000000; i++ ){}
                UART4->C2 |= UART_C2_RE(ENABLE);
                state = FIND_PREAMBLE;
                break;

            case FIND_MSG_ID:
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'G'){ state = FRAME_DELAY; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'P'){ state = FRAME_DELAY; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'R'){ state = FRAME_DELAY; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'M'){ state = FRAME_DELAY; }
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != 'C'){ state = FRAME_DELAY; }

                state = SAVE_TIME;
                break;

            case SAVE_TIME:
                while(RX_DATA_FLAG != SET){}
                if(UART4->D != ','){ state = FRAME_DELAY; }

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
                if(UART4->D != 'A'){ state = FRAME_DELAY; }

                for(int i = 0; i < 37; i++)
                {
                    while(RX_DATA_FLAG != SET){}
                    temp_read = UART4->D;
                }

                while(RX_DATA_FLAG != SET){}
                if(UART4->D != ','){ state = FRAME_DELAY; }

                for(int i = 6; i < 12; i++)
                {
                    while(RX_DATA_FLAG != SET){}
                    gpsMessage[i] = UART4->D;
                }

                state = TIME_SET;
                break;

            case TIME_SET:
                UART4->C2 &= ~(UART_C2_RE(ENABLE) | UART_C2_RIE(ENABLE));

                gpsTimeDateData.year  = (gpsMessage[10] - '0')*10U +
                                        (gpsMessage[11] - '0');

                gpsTimeDateData.month = (gpsMessage[8] - '0')*10U +
                                        (gpsMessage[9] - '0');

                gpsTimeDateData.day   = (gpsMessage[6] - '0')*10U +
                                        (gpsMessage[7] - '0');

                gpsTimeDateData.year  = (gpsMessage[10] - '0')*10U +
                                        (gpsMessage[11] - '0');

                gpsTimeDateData.month = (gpsMessage[8] - '0')*10U +
                                        (gpsMessage[9] - '0');

                gpsTimeDateData.hour  = (gpsMessage[0] - '0')*10U +
                                        (gpsMessage[1] - '0');

                gpsTimeDateData.min   = (gpsMessage[2] - '0')*10U +
                                        (gpsMessage[3] - '0');

                gpsTimeDateData.sec   = (gpsMessage[4] - '0')*10U +
                                        (gpsMessage[5] - '0');

                rx_status = DISABLED;
                break;

            default:
                break;
        }
    }
}

/******************************************************************************
*   gpsGetTimeTask() -
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
//void gpsGetTimeTask(void *pvParameters)
//{
//    TimeCompState_t state = MAJOR_DELAY;
//    uint8_t temp_read;
//
//    while(1)
//    {
//        switch(state)
//        {
//            case MAJOR_DELAY:
//                vTaskDelay(pdMS_TO_TICKS(GPSTIMECOMPTASK_WAIT_MS));
//                UART4->C2 |= (UART_C2_RE(ENABLE) | UART_C2_RIE(ENABLE));
//                state = FIND_PREAMBLE;
//                break;
//
//            case FIND_PREAMBLE:
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                state = ((UART4->D == '$') ? FIND_MSG_ID : FRAME_DELAY);
//                break;
//
//            case FRAME_DELAY:
//                UART4->C2 &= ~(UART_C2_RE(ENABLE) | UART_C2_RIE(ENABLE));
//                vTaskDelay(pdMS_TO_TICKS(500U));
//                state = FIND_PREAMBLE;
//                break;
//
//            case FIND_MSG_ID:
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != 'G'){ state = FRAME_DELAY; }
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != 'P'){ state = FRAME_DELAY; }
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != 'R'){ state = FRAME_DELAY; }
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != 'M'){ state = FRAME_DELAY; }
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != 'C'){ state = FRAME_DELAY; }
//
//                state = SAVE_TIME;
//                break;
//
//            case SAVE_TIME:
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != ','){ state = FRAME_DELAY; }
//
//                for(int i = 0; i < 6; i++)
//                {
//                    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                    gpsMessage[i] = UART->D;
//                }
//
//                for(int i = 0; i < 5; i++)
//                {
//                    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                    temp_read = UART->D;
//                }
//
//                ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                if(UART4->D != 'A'){ state = FRAME_DELAY; }
//
//                for(int i = 0; i < 38; i++)
//                {
//                    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                    temp_read = UART->D;
//                }
//
//                for(int i = 6; i < 12; i++)
//                {
//                    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
//                    gpsMessage[i] = UART->D;
//                }
//
//                state = TIME_COMP;
//                break;
//
//            case TIME_COMP:
//                UART4->C2 &= ~(UART_C2_RE(ENABLE) | UART_C2_RIE(ENABLE));
//
//                state = MAJOR_DELAY;
//                break;
//
//            default:
//                break;
//        }
//    }
//}

/******************************************************************************
*   UART4_RX_TX_IRQHandler() -
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
//void UART4_RX_TX_IRQHandler()
//{
//    BaseType_t xHigherPriorityTaskWoken;
//    xHigherPriorityTaskWoken = pdFALSE;
//
//    vTaskNotifyGiveFromISR(gpsTimeCompTaskHandle, &xHigherPriorityTaskWoken);
//
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
