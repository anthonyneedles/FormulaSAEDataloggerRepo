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

#define ENABLE                          0x01U
#define ALT_1_GPIO                      0x01U
#define ALT_2_I2C                       0x02U

/* Task parameters */
#define ACCELGYROSAMPLERTASK_PRIORITY      3U
#define ACCELGYROSAMPLERTASK_STKSIZE     256U

/* Input pins for accel/gyro's interrupt outputs, all PORTE. */
#define INT1_PIN_NUM                       9U
#define INT2_PIN_NUM                      12U

#define I2C3_SDA_PIN_NUM                  10U
#define I2C3_SCL_PIN_NUM                  11U
#define I2C3_SRC_CLK_HZ             60000000U
#define ACCELGYRO_I2C_ADDR              0x6BU
#define I2C_DATA_LENGTH                    1U

#define I2C3_BASE_PTR ((I2C_Type *)I2C3_BASE)

//static void accelgyroSamplerTask(void *);

//uint8_t accelgyroBuffer[I2C_DATA_LENGTH] = {0x01, 0x02};



void AccelGyroInit()
{
    i2c_direction_t dir = kI2C_Read;
    status_t status;
    uint8_t buf[I2C_DATA_LENGTH] = {0x0FU};
    i2c_master_transfer_t handle = {kI2C_TransferDefaultFlag,
                                    ACCELGYRO_I2C_ADDR,
                                    dir,
                                    0,
                                    0,
                                    buf,
                                    1U};
    i2c_master_config_t config;
    I2C_MasterGetDefaultConfig(&config);
    I2C_MasterInit(I2C3_BASE_PTR, &config, I2C3_SRC_CLK_HZ);

    PORTE->PCR[I2C3_SDA_PIN_NUM] = (PORT_PCR_MUX(ALT_2_I2C) |
                                    PORT_PCR_ODE(ENABLE));

    PORTE->PCR[I2C3_SCL_PIN_NUM] = (PORT_PCR_MUX(ALT_2_I2C) |
                                    PORT_PCR_ODE(ENABLE));

    I2C_Enable(I2C3_BASE_PTR, true);

    status = I2C_MasterTransferBlocking(I2C3_BASE_PTR, &handle);
    while(status != kStatus_Success){}


}

//void accelgyroSamplerTask(void *pvParameters)
//{
//
//
//    while(1)
//    {
//
//    }
//}
//
//void I2C3_IRQn()
//{
//
//}
