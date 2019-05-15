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
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"

#define ENABLE                      0x01U

/* Task parameters */
#define ACCELGYROSAMPLERTASK_PRIORITY     3U
#define ACCELGYROSAMPLERTASK_STKSIZE    256U

#define I2C3_SDA_PIN_NUM              10U
#define I2C3_SCL_PIN_NUM              11U
#define ALT_2_I2C                   0x02U
#define I2C3_SRC_CLK_HZ         60000000U
#define ACCELGYRO_I2C_ADDR          0x6BU
#define I2C_DATA_LENGTH                8U

#define I2C3_BASE_PTR ((I2C_Type *)I2C3_BASE)

static void accelgyroSamplerTask(void *);

uint8_t accelgyroBuffer[I2C_DATA_LENGTH] = { 0x01U, 0x02U, 0x03U, 0x04U,
                                             0x05U, 0x06U, 0x07U, 0x08U };


static i2c_master_handle_t *accelgyroMasterHandle;
static i2c_rtos_handle_t accelgyroRTOSHandle;
static i2c_master_transfer_t accelgyroMasterTransfer;

void AccelGyroInit()
{
    BaseType_t task_create_return;
    i2c_master_config_t master_config;
    status_t status;

    SIM->SCGC5 |= SIM_SCGC5_PORTE(ENABLE);
    SIM->SCGC1 |= SIM_SCGC1_I2C3(ENABLE);

    PORTE->PCR[I2C3_SDA_PIN_NUM] = (PORT_PCR_MUX(ALT_2_I2C) |
                                    PORT_PCR_ODE(ENABLE));

    PORTE->PCR[I2C3_SCL_PIN_NUM] = (PORT_PCR_MUX(ALT_2_I2C) |
                                    PORT_PCR_ODE(ENABLE));

    task_create_return = xTaskCreate(accelgyroSamplerTask,
                                     "Accel/Gyro Sampler Task",
                                     ACCELGYROSAMPLERTASK_PRIORITY,
                                     NULL,
                                     ACCELGYROSAMPLERTASK_STKSIZE,
                                     NULL);

    while(task_create_return == pdFAIL){ /* Error trap */ }

    /* masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true; */
    I2C_MasterGetDefaultConfig(&master_config);

    status = I2C_RTOS_Init(&accelgyroRTOSHandle,
                           I2C3_BASE_PTR,
                           &master_config,
                           I2C3_SRC_CLK_HZ);

    while (status != kStatus_Success){ /* Error trap */ }

    accelgyroMasterHandle = &accelgyroRTOSHandle.drv_handle;
    memset(&accelgyroMasterTransfer, 0, sizeof(accelgyroMasterTransfer));
}

void accelgyroSamplerTask(void *pvParameters)
{
    status_t status;

    accelgyroMasterTransfer.slaveAddress = ACCELGYRO_I2C_ADDR;
    accelgyroMasterTransfer.direction = kI2C_Write;
    accelgyroMasterTransfer.subaddress = 0;
    accelgyroMasterTransfer.subaddressSize = 0;
    accelgyroMasterTransfer.data = accelgyroBuffer;
    accelgyroMasterTransfer.dataSize = I2C_DATA_LENGTH;
    accelgyroMasterTransfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_RTOS_Transfer(&accelgyroRTOSHandle, &accelgyroMasterTransfer);
    while (status != kStatus_Success){ /* Error trap */ }

    while(1)
    {

    }
}
