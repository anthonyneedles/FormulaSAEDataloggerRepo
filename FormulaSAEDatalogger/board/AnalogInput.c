/******************************************************************************
*   AnalogInput.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Analog Input module of the Formula SAE Datalogger provides public
*   functions to set the power supply (5V/12V) provided to the 4 analog
*   sensors as well as the required input conditioning MUX selection. The
*   supplies are rated to supply a maximum of 20mA a either 5V or 12V per
*   sensor.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/01/2019
*
*   Created on: 04/29/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "AnalogInput.h"
#include "FreeRTOS.h"
#include "semphr.h"

/******************************************************************************
*   PRIVATE DEFINITIONS
******************************************************************************/
#define ENABLE                      0x01U
#define ALT_1_GPIO                  0x01U
#define OUTPUT                      0x01U
#define BIT_0_MASK        ((uint8_t)0x01U)

#define AIN1_POWER_PIN_NUM            23U
#define AIN2_POWER_PIN_NUM            22U
#define AIN3_POWER_PIN_NUM            21U
#define AIN4_POWER_PIN_NUM            20U

#define AIN1_COND_PIN_NUM             13U
#define AIN2_COND_PIN_NUM             12U
#define AIN3_COND_PIN_NUM             15U
#define AIN4_COND_PIN_NUM             14U

/******************************************************************************
*   PRIVATE MACROS
******************************************************************************/
#define AIN1_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN1_POWER_PIN_NUM))
#define AIN2_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN2_POWER_PIN_NUM))
#define AIN3_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN3_POWER_PIN_NUM))
#define AIN4_POWER_SET(x) (GPIOB->PDOR |= ((x) << AIN4_POWER_PIN_NUM))

#define AIN1_BIT(x) (((x) >> ((uint8_t)0U)) & BIT_0_MASK)
#define AIN2_BIT(x) (((x) >> ((uint8_t)1U)) & BIT_0_MASK)
#define AIN3_BIT(x) (((x) >> ((uint8_t)2U)) & BIT_0_MASK)
#define AIN4_BIT(x) (((x) >> ((uint8_t)3U)) & BIT_0_MASK)

/******************************************************************************
*   PRIVATE STRUCTURES
******************************************************************************/
typedef struct AnlgInData_t
{
    uint8_t state_field;
    uint16_t ain1_data;
    uint16_t ain2_data;
    uint16_t ain3_data;
    uint16_t ain4_data;
} AnlgInData_t;

/******************************************************************************
*   PRIVATE VARIABLES
******************************************************************************/
static AnlgInData_t anlginCurrentData;

/******************************************************************************
*   AnlgInInit() - Public function to
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void AnlgInInit()
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB(ENABLE);
    SIM->SCGC5 |= SIM_SCGC5_PORTC(ENABLE);

    PORTB->PCR[AIN1_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[AIN2_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[AIN3_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTB->PCR[AIN4_POWER_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);

    PORTC->PCR[AIN1_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTC->PCR[AIN2_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTC->PCR[AIN3_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);
    PORTC->PCR[AIN4_COND_PIN_NUM] = PORT_PCR_MUX(ALT_1_GPIO);

    GPIOB->PDDR |= GPIO_PDDR_PDD((OUTPUT << AIN1_POWER_PIN_NUM) |
                                 (OUTPUT << AIN2_POWER_PIN_NUM) |
                                 (OUTPUT << AIN3_POWER_PIN_NUM) |
                                 (OUTPUT << AIN4_POWER_PIN_NUM));

    GPIOC->PDDR |= GPIO_PDDR_PDD((OUTPUT << AIN1_COND_PIN_NUM) |
                                 (OUTPUT << AIN2_COND_PIN_NUM) |
                                 (OUTPUT << AIN3_COND_PIN_NUM) |
                                 (OUTPUT << AIN4_COND_PIN_NUM));

    anlginCurrentData.state_field = 0x00;
    anlginCurrentData.ain1_data = 0x0000;
    anlginCurrentData.ain2_data = 0x0000;
    anlginCurrentData.ain3_data = 0x0000;
    anlginCurrentData.ain4_data = 0x0000;
}

/******************************************************************************
*   AnlgInSet() - Public function to
*
*   Parameters:
*
*       AnlgInMsg_t msg - Message structure received from telemetry unit
*       with 8 bit power and state field (msg.power_state_field) and 32 bit
*       sampling rates field (msg.sampling_rate_field).
*
*       msg.power_state_field[0:3] corresponds to AIN[1:4] power, with a bit of
*       0 representing a power output of 5V (1 = 12V).
*
*       msg.power_state_field[4:7] corresponds to AIN[1:4] state, with a bit of
*       0 representing a non-active sensor input (1 = active).
*
*       msg.sampling_rate_field[0:7] corresponds to AIN1 sampling rate,
*       msg.sampling_rate_field[8:15] corresponds to AIN2 sampling rate, etc.
*
*   Return: None
******************************************************************************/
void AnlgInSet(AnlgInMsg_t msg)
{
    /* Convert state/power message to individual state/power bits for each
     * DOUT (i.e. msg.state_field bit 0 corresponds to DOUT1's desired state,
     * msg.state_field bit 1 corresponds to DOUT2's state, etc.) */
    AIN1_POWER_SET(AIN1_BIT(msg.power_state_field));
    AIN2_POWER_SET(AIN2_BIT(msg.power_state_field));
    AIN3_POWER_SET(AIN3_BIT(msg.power_state_field));
    AIN4_POWER_SET(AIN4_BIT(msg.power_state_field));
}

