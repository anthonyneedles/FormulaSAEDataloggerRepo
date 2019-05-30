/******************************************************************************
*   AccelGyro.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Accel/Gyro module of the Formula SAE Datalogger initializes and
*   establishes an I2C communication link with the on-board accelerometer and
*   gyroscope unit. The relevant data is then read from the unit and made
*   available other peripherals.
*
*   Accelerometer/Gyroscope Unit: LSM6DS3
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/29/2019
*
*   Created on: 05/15/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "AccelGyro.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Debug.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                          0x01U
#define ALT_1_GPIO                      0x01U
#define ALT_2_I2C                       0x02U
#define ACK_FOUND                          0U
#define SET                                1U
#define I2C3_SRC_CLK_HZ             60000000U
#define I2C_BUFFER_SIZE                   12U
#define PIT2                            0x02U
#define PIT_208HZ_LDVAL               288461U
#define PIT_104HZ_LDVAL               576922U
#define PIT_52HZ_LDVAL               1153845U
#define PIT_26HZ_LDVAL               2307691U

#define AGSAMPLERTASK_PRIORITY             8U
#define AGSAMPLERTASK_STKSIZE            256U

#define FREQ_DIV_VAL_104KHZ   ((uint8_t)0x9CU)
#define FREQ_DIV_VAL_417KHZ   ((uint8_t)0x53U)

/* PORTE */
#define I2C3_SDA_PIN_NUM                  10U
#define I2C3_SCL_PIN_NUM                  11U

/* LSM6DS3 slave address, with and without R/~W bit. */
#define ADDR_SLAVE            ((uint8_t)0x6BU)
#define ADDR_SLAVE_WRITE ((ADDR_SLAVE << 1) & ~((uint8_t)0x01U))
#define ADDR_SLAVE_READ  ((ADDR_SLAVE << 1) |  ((uint8_t)0x01U))

/* LSM6DS3 register sub-addresses. */
#define ADDR_WHO_AM_I                   0x0FU
#define ADDR_DATA_START                 0x22U
#define ADDR_CTRL1_H                    0x10U
#define ADDR_CTRL2_G                    0x11U
#define ADDR_INT1_CTRL                  0x0DU

/* +-4G accel range, 200Hz BW AA filter. */
#define CTRL1_XL_LOW_NIB       ((uint8_t)0x9U)

/* 500 dps gyro. */
#define CTRL2_G_LOW_NIB        ((uint8_t)0x4U)

/* Sampling rate configuration for accel/gyro.
 * See AGSet() header for values. */
#define SAMP_RATE_26HZ        ((uint8_t)0x00U)
#define SAMP_RATE_52HZ        ((uint8_t)0x01U)
#define SAMP_RATE_104HZ       ((uint8_t)0x02U)
#define SAMP_RATE_208HZ       ((uint8_t)0x03U)
#define SAMP_RATE_REG_OFFSET  ((uint8_t)0x02U)

/* Status flags. */
#define I2C_RX_ACK_FLAG ((I2C3->S & I2C_S_RXAK_MASK) >> I2C_S_RXAK_SHIFT)
#define I2C_BUS_BUSY_FLAG ((I2C3->S & I2C_S_BUSY_MASK) >> I2C_S_BUSY_SHIFT)
#define I2C_INTERRUPT_FLAG ((I2C3->S & I2C_S_IICIF_MASK) >> I2C_S_IICIF_SHIFT)

/* Blocks for ~4us to meet I2C timing requirements. */
#define I2C_BUS_FREE_DELAY() for(uint8_t i = 0; i < 70; i++){}

/* Enumerations for accel/gyro I2C communication state machine. */
typedef enum{
    IDLE,
    SEND_INIT_START,
    WRITE_SUB_ADDR,
    SEND_REPEAT_START,
    READ_DATA,
    FORMAT_DATA
} ag_i2c_states_t;

/******************************************************************************
*   Private Variables
******************************************************************************/
/* Data structure that will hold all Accel/Gyro data and configurations.
 * Secured by Mutex "agCurrentDataKey". */
static ag_data_t agCurrentData;

/* Mutex key that will protect agCurrentData structure. Must be pended on
 * if writing/reading agCurrentData is desired to ensure synchronization. */
static SemaphoreHandle_t agCurrentDataKey;

/* Semaphore that will be posted by PIT2 at 100Hz so trigger a new data grab,
 * bringing the I2C drive state machine out of IDLE. */
static SemaphoreHandle_t agReadStartTrigger;

/* Task handle for Accel/Gyro sampler task, used by I2C3_IRQHandler() to post
 * task notification. */
static TaskHandle_t agSamplerTaskHandle = NULL;

/* States for the I2C driver state machine. */
static ag_i2c_states_t agI2CState;

/******************************************************************************
*   Private Function Prototypes
******************************************************************************/
static void agSamplerTask(void *);

static void agNACKFailureCheck(void);

static void agSamplingRateChangeCheck(void);

static void agBusBusyCheck(void);

static void agPendOnInterrupt(void);

static void agConfigSamplingRate(uint8_t);

static void agResurrectModule(void);

/******************************************************************************
*   AGInit() - Public function to configure I2C3 for reception of accelerometer
*   and gyroscope data. Configuration of Accel/Gyro unit requires setting of
*   output data rate, data range, anti-aliasing filter cutoff, etc. in the
*   internal Accel/Gyro registers.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void AGInit()
{
    BaseType_t task_create_return;

    /* Enable PORTE and I2C3 clocks */
    SIM->SCGC5 |= SIM_SCGC5_PORTE(ENABLE);
    SIM->SCGC1 |= SIM_SCGC1_I2C3(ENABLE);

    /* Port muxing for I2C pins */
    PORTE->PCR[I2C3_SDA_PIN_NUM] = (PORT_PCR_MUX(ALT_2_I2C) |
                                    PORT_PCR_ODE(ENABLE));

    PORTE->PCR[I2C3_SCL_PIN_NUM] = (PORT_PCR_MUX(ALT_2_I2C) |
                                    PORT_PCR_ODE(ENABLE));

    /* Set I2C3 to fast mode. */
    I2C3->F = FREQ_DIV_VAL_417KHZ;

    /* PIT2 initialization for 5ms period, 100Hz trigger frequency. */
    SIM->SCGC6 |= SIM_SCGC6_PIT(ENABLE);
    PIT->MCR &= ~PIT_MCR_MDIS(ENABLE);
    PIT->CHANNEL[PIT2].LDVAL = PIT_104HZ_LDVAL;
    PIT->CHANNEL[PIT2].TCTRL |= (PIT_TCTRL_TIE(ENABLE) | PIT_TCTRL_TEN(ENABLE));

    /* Initial configuration: Sampling rate = 104Hz (BW = 52Hz). */
    agCurrentData.sampling_rate = SAMP_RATE_104HZ;
    agCurrentData.accel_data.x = (uint16_t)0x0000U;
    agCurrentData.accel_data.y = (uint16_t)0x0000U;
    agCurrentData.accel_data.z = (uint16_t)0x0000U;
    agCurrentData.gyro_data.x  = (uint16_t)0x0000U;
    agCurrentData.gyro_data.y  = (uint16_t)0x0000U;
    agCurrentData.gyro_data.z  = (uint16_t)0x0000U;

    agI2CState = IDLE;

    agCurrentDataKey = xSemaphoreCreateMutex();
    while(agCurrentDataKey == NULL){ /* Out of heap memory (DEBUG TRAP). */ }

    agReadStartTrigger = xSemaphoreCreateBinary();
    while(agReadStartTrigger == NULL){ /* Out of heap memory (DEBUG TRAP). */ }

    task_create_return = xTaskCreate(agSamplerTask,
                                     "Accel/Gyro Sampler Task",
                                     AGSAMPLERTASK_STKSIZE,
                                     NULL,
                                     AGSAMPLERTASK_PRIORITY,
                                     &agSamplerTaskHandle);

    while(task_create_return == pdFAIL){ /* Out of heap memory (DEBUG TRAP). */ }

    NVIC_SetPriority(I2C3_IRQn, 2U);
    NVIC_ClearPendingIRQ(I2C3_IRQn);
    NVIC_EnableIRQ(I2C3_IRQn);

    NVIC_SetPriority(PIT2_IRQn, 2U);
    NVIC_ClearPendingIRQ(PIT2_IRQn);
    NVIC_EnableIRQ(PIT2_IRQn);
}

/******************************************************************************
*   agSamplerTask() - This FreeRTOS task consists of a state machine for a
*   non-blocking, FreeRTOS friendly I2C driver. This specialized driver consists
*   of an I2C transfer via initial start, 7-bit slave address write, 8-bit data
*   sub-address start write, a repeated start and slave address write, then 12
*   repeated reads. The read data will then be written to necessary structure
*   element of agCurrentData.
*
*   IDLE - Pends on semaphore that will be posted by PIT2 IRQ at 100Hz. Performs
*   line busy check (which should never happen at this point), and communicates
*   to the accel/gyro unit to configure sampling rates if a change is desired.
*   Once PIT2 trigger is posted, state machine will transfer to reading states.
*
*   SEND_INIT_START - Sends the first start signal, followed by the slave
*   address, in WRITE mode. The slave will acknowledge and anticipate a byte
*   write.
*
*   WRITE_SUB_ADDR - Sends sub-address to accel/gyro to prepare to read the
*   data from this address. While this does does write a value to an existing
*   register, it basically tells the slave the address of the register to
*   start transmitting data from if a read command is sent.
*
*   SEND_REPEAT_START - Sends the repeated start signal, which allows a quicker
*   data transfer as the alternative would be to send a stop signal, then resend
*   the initial start signal. Followed by slave address, in READ mode.
*
*   READ_DATA - Reads dummy byte to release the SDA bus. Reads data simply
*   by reading value in I2C3 data register. The slave automatically increments
*   the sub-address. Since all accel/gyro data are in sequential sub-addresses,
*   no further writes to the slave are needed. For the final byte read, the
*   master must send a NACK on the 9th clock, send a stop signal, then finally
*   read the last data byte. At this point all data is saved in the Rx buffer.
*
*   FORMAT_DATA - Converts the 8-bit high and 8-bit low data read from the
*   accel/gyro to the complete 16-bit value. This value is in 2s complement.
*   The order of data in agRxBuffer[0:11] is:
*
*   accel.x.low, accel.x.high, accel.y.low, accel.y.high, accel.z.low, accel.z.high,
*   gyro.x.low, gyro.x.high, gyro.y.low, gyro.y.high, gyro.z.low, and gyro.z.high.
*
*   ERROR - Disables I2C3 and PIT2 interrupts, and deletes accel/gyro sampler
*   task. This effectively disables this entire module. No signal is given to
*   the Telemetry module or SDcard module. Accel/gyro data is still available,
*   but set to 0.
*
*   Parameters:
*
*       void *pvParameters - A value that will passed into the task when it is
*       created as the task's parameter. Not used for this task.
*
*   Return: None
******************************************************************************/
static void agSamplerTask(void *pvParameters)
{
    uint8_t dummy = 0;
    uint8_t agRxBuffer[I2C_BUFFER_SIZE];

    /* Get rid of warnings. */
    dummy++;

    /* Enable I2C3 and I2C3 interrupts. */
    I2C3->C1 = (I2C_C1_IICEN(ENABLE) | I2C_C1_IICIE(ENABLE));

    while(1)
    {
        switch(agI2CState)
        {
            case IDLE:
                agI2CState = SEND_INIT_START;

                xSemaphoreTake(agReadStartTrigger, portMAX_DELAY);

                agBusBusyCheck();

                /* If a change in sampling rate is desired, write to accel/gyro. */
                agSamplingRateChangeCheck();
                break;

            case SEND_INIT_START:
                agI2CState = WRITE_SUB_ADDR;

                /* Set TX mode and send start signal followed by slave address
                 * with cleared (writing to slave) R/!W bit (LSB). */
                I2C3->C1 |= (I2C_C1_TX(ENABLE) | I2C_C1_MST(ENABLE));
                I2C3->D = ADDR_SLAVE_WRITE;

                agPendOnInterrupt();
                break;

            case WRITE_SUB_ADDR:
                agI2CState = SEND_REPEAT_START;

                /* "Write" data sub-address to slave, making the following read
                 * be the data in the register (accel x-axis lower byte). */
                I2C3->D = ADDR_DATA_START;

                agPendOnInterrupt();

                agNACKFailureCheck();
                break;

            case SEND_REPEAT_START:
                agI2CState = READ_DATA;

                /* Send repeated start signal followed by slave address with set
                 * (reading from slave) R/!W bit (LSB). */
                I2C3->C1 |= I2C_C1_RSTA_MASK;
                I2C3->D = ADDR_SLAVE_READ;

                agPendOnInterrupt();
                break;

            case READ_DATA:
                agI2CState = FORMAT_DATA;

                /* Set RX mode and read the data register into a dummy value. */
                I2C3->C1 &= ~I2C_C1_TX_MASK;
                dummy = I2C3->D;

                /* Read first 11 bytes with no master provided NACK. */
                for(uint8_t buf_index = 0; buf_index < (I2C_BUFFER_SIZE - 1); buf_index++)
                {
                    agPendOnInterrupt();
                    agRxBuffer[buf_index] = I2C3->D;
                }

                /* The master receiver must send a NACK after the last read
                 * byte, right before the STOP signal. */
                I2C3->C1 |= I2C_C1_TXAK_MASK;

                agPendOnInterrupt();

                /* Send stop signal. */
                I2C3->C1 &= ~I2C_C1_MST_MASK;

                /* Read last byte, with master provided NACK. Must be done after
                 * stop signal is sent. */
                agRxBuffer[I2C_BUFFER_SIZE - 1] = I2C3->D;

                I2C3->C1 |= I2C_C1_TX_MASK;
                I2C3->C1 &= ~I2C_C1_TXAK_MASK;
                break;

            case FORMAT_DATA:
                agCurrentData.gyro_data.x = ((((uint16_t)agRxBuffer[1]) << 8) |
                                              ((uint16_t)agRxBuffer[0]));

                agCurrentData.gyro_data.y = ((((uint16_t)agRxBuffer[3]) << 8) |
                                              ((uint16_t)agRxBuffer[1]));

                agCurrentData.gyro_data.z = ((((uint16_t)agRxBuffer[5]) << 8) |
                                              ((uint16_t)agRxBuffer[4]));

                agCurrentData.accel_data.x = ((((uint16_t)agRxBuffer[7]) << 8) |
                                               ((uint16_t)agRxBuffer[5]));

                agCurrentData.accel_data.y = ((((uint16_t)agRxBuffer[9]) << 8) |
                                               ((uint16_t)agRxBuffer[8]));

                agCurrentData.accel_data.z = ((((uint16_t)agRxBuffer[11]) << 8) |
                                               ((uint16_t)agRxBuffer[10]));

                agI2CState = IDLE;
                break;

            default:
                break;
        }
    }
}

/******************************************************************************
*   I2C3_IRQHandler() - Interrupt handler for I2C3 IICIF. This triggers upon
*   byte transfer completion (including ACK/NACK). Clears IICIF flag and posts
*   task notification to Accel/Gyro Sampler Task. This allows non-blocking I2C
*   driver operation.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void I2C3_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    I2C3->S |= I2C_S_IICIF_MASK;
    vTaskNotifyGiveFromISR(agSamplerTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   PIT2_IRQHandler() - Interrupt handler for PIT2 TIF. Will enter every 10ms
*   (100Hz) to trigger a new accel/gyro data read via I2C3 driver.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void PIT2_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    PIT->CHANNEL[PIT2].TFLG |= PIT_TFLG_TIF_MASK;

    xSemaphoreGiveFromISR(agReadStartTrigger, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/******************************************************************************
*   AGSet() - Public function to set accel/gyro sampling rate and sampling
*   on/off state to achieve requested results from received user message. Saves
*   the new configuration data from the message in Mutex protected private data
*   structure.
*
*   Parameters:
*
*       AnlgInMsg_t msg - Message structure received from telemetry unit
*       with 8 bit power and state field (msg.power_state_field) and 32 bit
*       sampling rates field (msg.sampling_rate_field).
*
*       msg.sampling_rate_field[2:0] corresponds to the requested sample rate
*       of the accel/gyro. Valid values are:
*
*           00b = SR of 26Hz,  0010b written to AG register
*           01b = SR of 52Hz,  0011b written to AG register
*           10b = SR of 104Hz, 0100b written to AG register
*           11b = SR of 208Hz, 0101b written to AG register
*
*   Return: None
******************************************************************************/
void AGSet(ag_msg_t msg)
{
    if(msg.sampling_rate_field <= 0x4)
    {
        /* Pend on Mutex to update data structure */
        xSemaphoreTake(agCurrentDataKey, portMAX_DELAY);

        agCurrentData.sampling_rate = msg.sampling_rate_field;

        xSemaphoreGive(agCurrentDataKey);
    } else{}
}

/******************************************************************************
*   AGGetData() - Public function to copy current accel/gyro data structure
*   for use in transmission/storage.
*
*   Parameters:
*
*       ag_data_t *ldata - Pointer to caller-side data structure which will
*       have current data copied to it.
*
*   Return: None
******************************************************************************/
void AGGetData(ag_data_t *ldata)
{
    /* Pend on Mutex to update data structure */
    xSemaphoreTake(agCurrentDataKey, portMAX_DELAY);

    *ldata = agCurrentData;

    xSemaphoreGive(agCurrentDataKey);
}

/******************************************************************************
*   agNACKFailureCheck() - Private function to that checks for the absence of
*   an ACK from the slave (NACK) on SDA bus during the 9th clock cycle. This
*   may be due to a failing synchronization of the master/slave. If this happens
*   the current transfer is abandoned and the state machine is restarted, unless
*   this happens 5 times during a session, then the ERROR state will be entered.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void agNACKFailureCheck()
{
    static uint8_t failure_count = 0;

    if(I2C_RX_ACK_FLAG != ACK_FOUND)
    { /* NACK found on 9th clock pulse. Increment failure counter,
       * send STOP signal, then try again. */
        failure_count++;
        I2C3->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
        agI2CState = IDLE;
    } else {}

    if(failure_count > 4)
    { /* Resurrect if 5 repeated failed attempts to receive an ACK occur. */
        agResurrectModule();
    } else{}
}

/******************************************************************************
*   agSamplingRateChangeCheck() - Private function that detects a change in the
*   requested accel/gyro sampling rate via the AGSet() function. If a change
*   is detected, the configuration routine will run, writing to the
*   accelerometer/gyroscope's internal registers.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void agSamplingRateChangeCheck()
{
    static uint8_t previous_sampling_rate = 0xFF;

    /* Pend on Mutex to update data structure. */
    xSemaphoreTake(agCurrentDataKey, portMAX_DELAY);

    if(agCurrentData.sampling_rate != previous_sampling_rate)
    {
        previous_sampling_rate = agCurrentData.sampling_rate;
        xSemaphoreGive(agCurrentDataKey);
        agConfigSamplingRate(agCurrentData.sampling_rate);
    } else
    {
        xSemaphoreGive(agCurrentDataKey);
    }
}

/******************************************************************************
*   agBusBusyCheck() - Private function to that checks if the SDA bus is busy.
*   This occurs upon failure to relinquish control by the master or slave, since
*   the bus is set to busy when a start signal is found and cleared when a
*   proceeding stop signal is found. Therefore, is the bus is busy when not
*   expected to be, the system should enter the ERROR state.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void agBusBusyCheck()
{
    if(I2C_BUS_BUSY_FLAG == SET)
    { /* Resurrect if bus is busy, this should never happen here. */
        agResurrectModule();
    } else {}
}

/******************************************************************************
*   agPendOnInterrupt() - Private function that pends on the I2C3 ISR. This will
*   trigger upon byte completion, so the task can enter an idle state and allow
*   the CPU to active other tasks.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void agPendOnInterrupt()
{
    uint8_t notify_count;

    /* Place task into idle state until I2C3 ISR notifies task. */
    notify_count = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    if(notify_count == 0)
    { /* Resurrect if failed to be notified. */
        agResurrectModule();
    } else {}
}

/******************************************************************************
*   agConfigSamplingRate() - Private function that writes to the accel/gyro config
*   registers to alter the sampling rate/bandwidth to a user specified value.
*   This function requires two blocking nulls loops for meeting I2C setup/hold
*   times. These are justified by being very short (microseconds) and with the
*   fact that this function will never be called at a high frequency.
*
*   Parameters:
*
*       uint8_t sample_rate - Sample rate value that will be written to the
*       LSM6DS3's CTRL1_XL ODR_XL[3:0] parameter and the LSM6DS3's CTRL1_G
*       ODR_G[3:0] parameter. These parameters set the accelerometer's and the
*       gyroscopes's output data rate (ODR), respectively.
*
*   Return: None
******************************************************************************/
static void agConfigSamplingRate(uint8_t sample_rate)
{
    /* Send start signal followed by slave address (write). */
    I2C3->C1 |= (I2C_C1_TX(ENABLE) | I2C_C1_MST(ENABLE));
    I2C3->D = ADDR_SLAVE_WRITE;

    agPendOnInterrupt();

    /* Write to accel config register first. */
    I2C3->D = ADDR_CTRL1_H;

    agPendOnInterrupt();

    /* Write to config reg to set requested sample rate. */
    I2C3->D = (((sample_rate + SAMP_RATE_REG_OFFSET) << 4) | CTRL1_XL_LOW_NIB);

    agPendOnInterrupt();

    /* Send stop signal. */
    I2C3->C1 &= ~I2C_C1_MST_MASK;

    /* Delay for bus free time. */
    I2C_BUS_FREE_DELAY();

    /* Send start signal followed by slave address (write). */
    I2C3->C1 |= (I2C_C1_TX(ENABLE) | I2C_C1_MST(ENABLE));
    I2C3->D = ADDR_SLAVE_WRITE;

    agPendOnInterrupt();


    /* Write to gyro config register next. */
    I2C3->D = ADDR_CTRL2_G;

    agPendOnInterrupt();

    /* Write to config reg to set requested sample rate. */
    I2C3->D = (((sample_rate + SAMP_RATE_REG_OFFSET) << 4) | CTRL2_G_LOW_NIB);

    agPendOnInterrupt();

    /* Send stop signal. */
    I2C3->C1 &= ~I2C_C1_MST_MASK;

    /* Delay for bus free time. */
    I2C_BUS_FREE_DELAY();

    /* Change PIT trigger frequency for sampler task. */
    switch(sample_rate)
    {
        case SAMP_RATE_26HZ:
            PIT->CHANNEL[PIT2].LDVAL = PIT_26HZ_LDVAL;
            break;

        case SAMP_RATE_52HZ:
            PIT->CHANNEL[PIT2].LDVAL = PIT_52HZ_LDVAL;
            break;

        case SAMP_RATE_104HZ:
            PIT->CHANNEL[PIT2].LDVAL = PIT_104HZ_LDVAL;
            break;

        case SAMP_RATE_208HZ:
            PIT->CHANNEL[PIT2].LDVAL = PIT_208HZ_LDVAL;
            break;

        default:
            break;
    }
}

/******************************************************************************
*   agResurrectModule() - Private function that attempts so solve an
*   error by reinitializing module. Disables preemption and interrupts of a
*   system priority <= configMAX_SYSCALL_INTERRUPT_PRIORITY (effectively, these
*   are interrupts set to have a priority <= 1 by NVIC_SetPriority()) when
*   deleting RTOS structures.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void agResurrectModule()
{
    taskENTER_CRITICAL();

    NVIC_DisableIRQ(I2C3_IRQn);
    NVIC_DisableIRQ(PIT2_IRQn);
    vSemaphoreDelete(agCurrentDataKey);
    vSemaphoreDelete(agReadStartTrigger);
    vTaskDelete(agSamplerTaskHandle);

    taskEXIT_CRITICAL();

    AGInit();
}
