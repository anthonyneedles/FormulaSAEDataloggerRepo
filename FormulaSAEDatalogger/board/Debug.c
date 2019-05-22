/******************************************************************************
*   Debug.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The debug module enables debug outputs for use of setting and clearing
*   in-line with code to get very precise timing measurements. When DebugInit()
*   is ran the board test points CLKOUT and RTCCLKOUT change from providing
*   internal clock signals to providing DB1 and DB2, respectively.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/22/2019
*
*   Created on: 05/21/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "Debug.h"

/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                         1U
#define ALT_1_GPIO                     1U
#define OUTPUT                         1U

/******************************************************************************
*   DebugInit() - Public function to configure PTC3 and PTE26 for general
*   purpose outputs.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DebugInit()
{
    SIM->SCGC5 |= (SIM_SCGC5_PORTC(ENABLE) | SIM_SCGC5_PORTE(ENABLE));

    PORTC->PCR[3] = ((PORTC->PCR[3] & ~PORT_PCR_MUX_MASK) |
                      PORT_PCR_MUX(ALT_1_GPIO));

    PORTE->PCR[26] = ((PORTE->PCR[26] & ~PORT_PCR_MUX_MASK) |
                       PORT_PCR_MUX(ALT_1_GPIO));

    GPIOC->PDDR |= GPIO_PDDR_PDD(OUTPUT << DB1_OUTPUT_PIN_NUM);
    GPIOE->PDDR |= GPIO_PDDR_PDD(OUTPUT << DB2_OUTPUT_PIN_NUM);

    GPIOC->PDOR |= GPIO_PDOR_PDO(ENABLE << DB1_OUTPUT_PIN_NUM);
    GPIOE->PDOR |= GPIO_PDOR_PDO(ENABLE << DB2_OUTPUT_PIN_NUM);
}
