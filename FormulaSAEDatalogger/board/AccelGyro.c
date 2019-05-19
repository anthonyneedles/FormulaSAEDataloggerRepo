/******************************************************************************
*   AccelGyro.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Analog Input module of the Formula SAE Datalogger provides public
*   functions to set the power supply (5V/12V) provided to the 4 analog
*   sensors as well as the required input conditioning MUX selection. The
*   supplies are rated to supply a maximum of 20mA a either 5V or 12V per
*   sensor. The analog signals will be measured via ADC1 channels, with PIT1
*   providing hardware triggering at 8kHz.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/15/2019
*
*   Created on: 04/29/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "AnalogInput.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "fsl_i2c_freertos.h"
#include "fsl_i2c.h"

void accelgyroSamplerTask(void *);

#define ENABLE                          0x01U
#define ALT_1_GPIO                      0x01U
#define ALT_2_I2C                       0x02U
#define ACK_FOUND                          0U
#define SET                                1U

/* Task parameters */
#define ACCELGYROSAMPLERTASK_PRIORITY      5U
#define ACCELGYROSAMPLERTASK_STKSIZE     256U

/* Input pins for accel/gyro's interrupt outputs, all PORTE. */
#define INT1_PIN_NUM                       9U
#define INT2_PIN_NUM                      12U

#define FREQ_DIV_VAL_104KHZ   ((uint8_t)0x9CU)
#define FREQ_DIV_VAL_417KHZ   ((uint8_t)0x53U)

#define I2C3_SDA_PIN_NUM                  10U
#define I2C3_SCL_PIN_NUM                  11U
#define I2C3_SRC_CLK_HZ             60000000U
#define ACCELGYRO_I2C_ADDR              0x6BU
#define I2C_BUFFER_SIZE                   16U

#define ADDR_WHO_AM_I                   0x0FU
#define ADDR_DATA_START                 0x22U
#define ADDR_CTRL1_H                    0x10U
#define ADDR_CTRL2_G                    0x11U

#define I2C_RX_ACK_FLAG ((I2C3->S & I2C_S_RXAK_MASK) >> I2C_S_RXAK_SHIFT)
#define I2C_BUS_BUSY_FLAG ((I2C3->S & I2C_S_BUSY_MASK) >> I2C_S_BUSY_SHIFT)

typedef enum{
    IDLE,
    SEND_INIT_START,
    WRITE_REGISTER,
    SEND_REPEAT_START,
    READ_DATA,
    ERROR
} accelgyroI2CStates_t;

typedef struct AccelGyroInData_t
{
    uint8_t accelgyro_state_field;
    uint8_t accel_samp_rate;
    uint8_t gyro_samp_rate;
    uint16_t accel_data;
    uint16_t gyro_data;
} AccelGyroInData_t;



/******************************************************************************
*   Private Variables
******************************************************************************/
/* Data structure that will hold all Analog Input data and configurations.
 * Secured by Mutex "anlginCurrentDataKey" */
static AccelGyroInData_t accelgyroCurrentData;

/* Mutex key that will protect anlginCurrentData structure. Must be pended on
 * if writing/reading anlginCurrentData is desired to ensure synchronization */
static SemaphoreHandle_t accelgyroCurrentDataKey;

/* Task handle for Analog In sampler task, used by ADC1_ISRHandler() to post
 * task notification */
static TaskHandle_t accelgyroSamplerTaskHandle = NULL;

void AccelGyroInit()
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

    I2C3->F = FREQ_DIV_VAL_417KHZ;
    I2C3->C1 = (I2C_C1_IICEN(ENABLE) | I2C_C1_IICIE(ENABLE));

    accelgyroCurrentDataKey = xSemaphoreCreateMutex();
    while(accelgyroCurrentDataKey == NULL){ /* Error trap */ }

    /* Creation of accelerometer/gyroscope sampling task. */
    task_create_return = xTaskCreate(accelgyroSamplerTask,
                                     "Accel/Gyro Sampler Task",
                                     ACCELGYROSAMPLERTASK_STKSIZE,
                                     NULL,
                                     ACCELGYROSAMPLERTASK_PRIORITY,
                                     &accelgyroSamplerTaskHandle);

    while(task_create_return == pdFAIL){ /* Error trap */ }



    I2C3->C1 |= (I2C_C1_TX(ENABLE) | I2C_C1_MST(ENABLE));
    I2C3->D = ((ACCELGYRO_I2C_ADDR << 1) & ~0x01U);

    while((I2C3->S & I2C_S_IICIF_MASK) == 0) {}
    I2C3->S |= I2C_S_IICIF(1);

    I2C3->D = ADDR_CTRL1_H;

    while((I2C3->S & I2C_S_IICIF_MASK) == 0) {}
    I2C3->S |= I2C_S_IICIF(1);

    I2C3->D = (uint8_t)0x55U;

    while((I2C3->S & I2C_S_IICIF_MASK) == 0) {}
    I2C3->S |= I2C_S_IICIF(1);

    I2C3->C1 &= ~I2C_C1_MST_MASK;

    for(int i = 0; i < 1000; i++){}

    I2C3->C1 |= (I2C_C1_TX(ENABLE) | I2C_C1_MST(ENABLE));
    I2C3->D = ((ACCELGYRO_I2C_ADDR << 1) & ~0x01U);

    while((I2C3->S & I2C_S_IICIF_MASK) == 0) {}
    I2C3->S |= I2C_S_IICIF(1);

    I2C3->D = ADDR_CTRL2_G;

    while((I2C3->S & I2C_S_IICIF_MASK) == 0) {}
    I2C3->S |= I2C_S_IICIF(1);

    I2C3->D = (uint8_t)0x50U;

    while((I2C3->S & I2C_S_IICIF_MASK) == 0) {}
    I2C3->S |= I2C_S_IICIF(1);

    I2C3->C1 &= ~I2C_C1_MST_MASK;


    NVIC_SetPriority(I2C3_IRQn, 2U);
    NVIC_ClearPendingIRQ(I2C3_IRQn);
    NVIC_EnableIRQ(I2C3_IRQn);
}

void I2C3_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    I2C3->S |= I2C_S_IICIF_MASK;

    vTaskNotifyGiveFromISR(accelgyroSamplerTaskHandle, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void accelgyroSamplerTask(void *pvParameters)
{
    uint32_t notify_count;
    BaseType_t take_return;
    uint8_t dumb_ass_read;
    uint8_t failure_count = 0;
    accelgyroI2CStates_t accelgyroI2CState = IDLE;
    uint8_t accelgyroRxBuffer[I2C_BUFFER_SIZE];

    while(1)
    {
        switch(accelgyroI2CState)
        {
            case IDLE:
                if(failure_count > 4)
                { /* Go to ERROR state if 5 repeated failed attempts to receive
                   * an ACK occur. */
                    accelgyroI2CState = ERROR;
                    break;
                }

                vTaskDelay(pdMS_TO_TICKS(1000U));

                if(I2C_BUS_BUSY_FLAG == SET)
                { /* Go to ERROR state if bus is busy, this should never happen
                   * here. */
                    accelgyroI2CState = ERROR;
                    break;
                }

                accelgyroI2CState = SEND_INIT_START;
                break;

            case SEND_INIT_START:
                /* Set TX mode and send start signal followed by slave address
                 * with cleared (writing to slave) R/!W bit (LSB). */
                I2C3->C1 |= (I2C_C1_TX(ENABLE) | I2C_C1_MST(ENABLE));
                I2C3->D = ((ACCELGYRO_I2C_ADDR << 1) & ~0x01U);

                /* Place task into idle state until I2C3 ISR notifies task. */
                notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1U));
                if(notify_count == 0)
                { /* If this takes longer than 1ms, go to ERROR state. */
                    accelgyroI2CState = ERROR;
                    break;
                }

                accelgyroI2CState = WRITE_REGISTER;
                break;

            case WRITE_REGISTER:
                I2C3->D = ADDR_DATA_START;

                /* Place task into idle state until I2C3 ISR notifies task. */
                notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1U));
                if(notify_count == 0)
                { /* If this takes longer than 1ms, go to ERROR state. */
                    accelgyroI2CState = ERROR;
                    break;
                }

                if(I2C_RX_ACK_FLAG != ACK_FOUND)
                { /* NACK found on 9th clock pulse. Increment failure counter,
                   * send STOP signal, then try again. */
                    failure_count++;
                    I2C3->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK);
                    accelgyroI2CState = IDLE;
                    break;
                }

                accelgyroI2CState = SEND_REPEAT_START;
                break;

            case SEND_REPEAT_START:
                /* Send repeated start signal followed by slave address with set
                 * (reading from slave) R/!W bit (LSB). */
                I2C3->C1 |= I2C_C1_RSTA_MASK;
                I2C3->D = ((ACCELGYRO_I2C_ADDR << 1) | 0x01U);

                /* Place task into idle state until I2C3 ISR notifies task. */
                notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1U));
                if(notify_count == 0)
                { /* If this takes longer than 1ms, go to ERROR state. */
                    accelgyroI2CState = ERROR;
                    break;
                }

                accelgyroI2CState = READ_DATA;
                break;

            case READ_DATA:
                /* Set RX mode and read the data register into a dummy value. */
                I2C3->C1 &= ~I2C_C1_TX_MASK;
                dumb_ass_read = I2C3->D;

                for(int buf_index = 0; buf_index < (I2C_BUFFER_SIZE - 1); buf_index++)
                {
                    /* Place task into idle state until I2C3 ISR notifies task. */
                    notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1U));
                    if(notify_count == 0)
                    { /* If this takes longer than 1ms, go to ERROR state. */
                        accelgyroI2CState = ERROR;
                        break;
                    }
                    accelgyroRxBuffer[buf_index] = I2C3->D;
                }

                /* The master receiver must send a NACK after the last read
                 * byte, right before the STOP signal. */
                I2C3->C1 |= I2C_C1_TXAK_MASK;

                /* Place task into idle state until I2C3 ISR notifies task. */
                notify_count = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1U));
                if(notify_count == 0)
                { /* If this takes longer than 1ms, go to ERROR state. */
                    accelgyroI2CState = ERROR;
                    break;
                }

                I2C3->C1 &= ~I2C_C1_MST_MASK;
                accelgyroRxBuffer[I2C_BUFFER_SIZE - 1] = I2C3->D;
                I2C3->C1 |= I2C_C1_TX_MASK;
                I2C3->C1 &= ~I2C_C1_TXAK_MASK;

                accelgyroI2CState = IDLE;
                break;

            case ERROR:
                while(1){}
                break;

            default:
                break;
        }
    }
}

