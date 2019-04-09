/******************************************************************************
*   ClockConfig.c.
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   This module handles initial clock configuration for external crystal High
*   Speed Run mode, external crystal RTC operation, test probe CLKOUTs, and
*   SDHC0 initialization. NXP clock config tool was used for reference. The
*   following clock frequencies are established:
*
*   Core/System Clock:      180MHz
*   Bus Clock:              60MHz
*   FlexBus/SDRAMC Clock:   60MHz
*   Flash Clock:            25.71MHz
*   OSCERCLK Clock:         12MHz
*   LPO Clock:              1kHz
*   RTC_CLKOUT (RTC1HzCLK): 1Hz
*   CLKOUT (OSCERCLK)       12MHz
*   SDHC Clock:             180MHz*
*
*   *This SDHC clock is considered an error by NXP clock config as it exceeds
*   the SDHC max frequency of 50MHz. However, as specified in SDHC_SYSCT, this
*   clock is further divided to produce a valid value.
*
*   Comments up to date as of: 04/08/2019
*
*   MCU: MK66FN2M0VLQ18R
*
*   Created on: 03/12/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef CLOCKCONFIG_H_
#define CLOCKCONFIG_H_

/******************************************************************************
*   ClockConfigRun() - Public function to run all private clock configuration
*   functions in valid order.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void ClockConfigRun(void);

#endif /* CLOCKCONFIG_H_ */
