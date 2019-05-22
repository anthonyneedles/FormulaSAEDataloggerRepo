/******************************************************************************
*   Debug.h
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
#ifndef DEBUG_H_
#define DEBUG_H_

/******************************************************************************
*   Public Definitions
******************************************************************************/
/* PORT C */
#define DB1_OUTPUT_PIN_NUM             3U

/* PORT E */
#define DB2_OUTPUT_PIN_NUM      26U

#define DB1_OUTPUT_SET() GPIOC->PSOR |= GPIO_PSOR_PTSO(1U << DB1_OUTPUT_PIN_NUM)
#define DB1_OUTPUT_CLEAR() GPIOC->PCOR |= GPIO_PCOR_PTCO(1U << DB1_OUTPUT_PIN_NUM)
#define DB1_OUTPUT_TOGGLE() GPIOC->PTOR |= GPIO_PTOR_PTTO(1U << DB1_OUTPUT_PIN_NUM)

#define DB2_OUTPUT_SET() GPIOE->PSOR |= GPIO_PSOR_PTSO(1U << DB2_OUTPUT_PIN_NUM)
#define DB2_OUTPUT_CLEAR() GPIOE->PCOR |= GPIO_PCOR_PTCO(1U << DB2_OUTPUT_PIN_NUM)
#define DB2_OUTPUT_TOGGLE() GPIOE->PTOR |= GPIO_PTOR_PTTO(1U << DB2_OUTPUT_PIN_NUM)

/******************************************************************************
*   Public Function Prototypes
******************************************************************************/
/******************************************************************************
*   DebugInit() - Public function to configure PTC3 and PTE26 for general
*   purpose outputs.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void DebugInit(void);

#endif /* DEBUG_H_ */
