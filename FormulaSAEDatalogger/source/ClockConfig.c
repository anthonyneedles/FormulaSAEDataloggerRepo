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
*   MCGFFCLK:               375kHz
*   Flash Clock:            25.71MHz
*   OSCERCLK Clock:         12MHz
*   LPO Clock:              1kHz
*   RTC_CLKOUT:             1Hz
*   CLKOUT (OSCERCLK)       12MHz
*   SDHC Clock:             180MHz*
*
*   *This SDHC clock is considered an error by NXP clock config as it exceeds
*   the SDHC max frequency of 50MHz. However, as specified in SDHC_SYSCT, this
*   clock is further divided to produce a valid value.
*
*   Comments up to date as of: 03/17/2019
*
*   MCU: MK66FN2M0VLQ18R
*
*   Created on: 03/12/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "FreeRTOS.h"
#include "ClockConfig.h"
/******************************************************************************
*   Private function prototypes
******************************************************************************/
static void clockConfigSetHSRun(void);
static void clockConfigSafeDivisions(void);
static void clockConfigSetXtalRTC(void);
static void clockConfigSetXtalOSC(void);
static void clockConfigSetMCG(void);
static void clockConfigSetSIM(void);
static void clockConfigSetCoreClock(void);
static void clockConfigSetCLKOUTs(void);
static void clockConfigSetSDHC0(void);
/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                      1U
#define DISABLE                     0U
#define HSRUN                       3U
#define PWRMODE_HSRUN            0x80U
#define SAFE_DIVISIONS     0x02260000U
#define VH_FREQ_RANGE               3U
#define FRDRIV_32                   0U
#define EXT_REF                   0x2U
#define OSC_READY                 0x1U
#define VMLTPY_30                  14U
#define PLL_CLOCK                 0x1U
#define PLL_LOCKED                0x1U
#define PLL_SEL                     1U
#define PLLCS_SOURCE              0x1U
#define OUTDIV1_1                   0U
#define OUTDIV2_3                   2U
#define OUTDIV3_3                   2U
#define OUTDIV4_7                   2U
#define MCGPLLCLK_SEL               1U
#define RTC32_SEL                   2U
#define OSCERCLK0_SEL               6U
#define RTC32K_SEL                  1U
#define CORE_CLK_SEL                0U
#define LOW_POWER_SEL               0U
#define CRYSTL_SEL                  1U
#define PLLFLL_SEL                  0U
#define CORE_CLOCK_HZ       180000000U

#define POWER_MODE_STATUS   (SMC->PMSTAT)
#define OSC_STATUS          ((MCG->S & MCG_S_OSCINIT0_MASK) >> MCG_S_OSCINIT0_SHIFT)
#define FLL_REF_STATUS      ((MCG->S & MCG_S_IREFST_MASK) >> MCG_S_IREFST_SHIFT)
#define PLLS_SOURCE_STATUS  ((MCG->S & MCG_S_PLLST_MASK) >> MCG_S_PLLST_SHIFT)
#define PLL_LOCK_STATUS     ((MCG->S & MCG_S_LOCK0_MASK) >> MCG_S_LOCK0_SHIFT)
#define PLL_SELECT_STATUS   ((MCG->S & MCG_S_PLLST_MASK) >> MCG_S_PLLST_SHIFT)
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern uint32_t SystemCoreClock;
/******************************************************************************
*   ClockConfigRun() - Public function to run all private clock configuration
*   functions in valid order.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void ClockConfigRun()
{
    clockConfigSetHSRun();
    clockConfigSafeDivisions();
    clockConfigSetXtalRTC();
    clockConfigSetXtalOSC();
    clockConfigSetMCG();
    clockConfigSetSIM();
    clockConfigSetCoreClock();
    clockConfigSetCLKOUTs();
    clockConfigSetSDHC0();
}
/******************************************************************************
*   clockConfigSetHSRun() - Private function to enable system for HSRUN mode.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetHSRun()
{
    /* Enable system to allow HSRUN mode. PMPROT can only be written to once
     * after any system reset */
    SMC->PMPROT |= SMC_PMPROT_AHSRUN(ENABLE);
    /* Enable HSRUN mode. */
    SMC->PMCTRL |= SMC_PMCTRL_RUNM(HSRUN);
    /* Wait until power mode status is set. */
    while(POWER_MODE_STATUS != PWRMODE_HSRUN){}
}
/******************************************************************************
*   clockConfigSafeDivisions() - Private Function to ensure that during
*   succeeding MCG output changes the core clock, bus clock, flexbus clock and
*   flash clock will remain in allowed range.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSafeDivisions()
{
    /* Set system level clock divisions OUTDIV1, OUTDIV2, OUTDIV3, and OUTDIV4.
     * */
    SIM->CLKDIV1 = SAFE_DIVISIONS;
}
/******************************************************************************
*   clockConfigSetXtalRTC() - Private function to set RTC module for peripheral
*   output and enabling 32k oscillator. Must wait 1000ms before enabling time
*   counter to ensure oscillator stabilization.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetXtalRTC()
{
    /* Allow access and interrupts for RTC. */
    SIM->SCGC6 |= SIM_SCGC6_RTC(ENABLE);
    /* Set 32k clock output to peripherals and enable 32k oscillator.  */
    RTC->CR |= ((RTC->CR & ~RTC_CR_CLKO_MASK) | RTC_CR_OSCE(ENABLE));
    /* Disallow access and interrupts for RTC. */
    SIM->SCGC6 |= SIM_SCGC6_RTC(DISABLE);
}
/******************************************************************************
*   clockConfigSetXtalOSC() - Private function to enable external reference
*   clock in Connection 3 (external capacitors, internal feedback resistor).
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetXtalOSC()
{
    /* Enables OSC to enable external reference clock. */
    OSC->CR |= OSC_CR_ERCLKEN(ENABLE);
}
/******************************************************************************
*   clockConfigSetMCG() - Private function to configure the MCG for desired
*   operation of PEE (PLL Engaged External). An external source (external
*   crystal & system oscillator) engaged to the PLL supplying MCGOUTCLK. This
*   will achieve a clock value of 180MHz on MCGOUTCLK to provide the SIM.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetMCG()
{
    /* MCG comes out of reset in FEI (FLL Engaged Internal) mode. The desired
     * mode is PEE. The MCG must first transition from FEI to FBE (FLL Bypass
     * External), then transition from FBE to PBE (PLL Bypass External), and
     * finally from PBE to PEE. */

    /* Transition to FBE: */

    /* Config MCG for very high frequency range (due to 12MHz external crystal),
     * low power operation, and a crystal external reference. */
    MCG->C2 = (MCG_C2_EREFS(CRYSTL_SEL) | MCG_C2_HGO(LOW_POWER_SEL) |
               MCG_C2_RANGE(VH_FREQ_RANGE));
    /* MCGOUTCLOCK will (temporarily) output external reference clock (12MHz).
     * FLL external reference divider will be the lowest value as the 31.25kHz to
     * 39.0625kHz range is only required to be met when the MCGOUTCLK source
     * is FLL. */
    MCG->C1 = (MCG_C1_CLKS(EXT_REF) | MCG_C1_FRDIV(FRDRIV_32));
    /* Wait until crystal oscillator stabilizes. */
    while(OSC_STATUS != OSC_READY){}
    /* Wait until source of FLL reference clock is external reference clock
     * (this does not immediately happen upon setting field in C2). */
    while(FLL_REF_STATUS != EXT_REF){}

    /* Transition to PBE: */

    /* PLL will multiply by 30 to produce a MCGPLL0CLK2X frequency of 12MHz*30 =
     * 360MHz. This will then be divided by 2 to achieve MCGPLL0CLK, 180MHZ.
     * The PLL is chosen as input to PLLS for later use as MCGOUTCLK. */
    MCG->C6 = (MCG_C6_PLLS(PLL_SEL) | MCG_C6_VDIV(VMLTPY_30));
    /* Wait until PLLS source of PLL synchronization. */
    while(PLLS_SOURCE_STATUS != PLL_CLOCK){}
    /* Wait until PLL has obtained lock and MCGPLLCLK is no longer gated off */
    while(PLL_LOCK_STATUS != PLL_LOCKED){}

    /* Transition to PEE: */

    /* Configure MCGOUTCLK for source of FLL/PLLCS (in this case 180MHz PLL). */
    MCG->C1 = ((MCG->C1 & ~MCG_C1_CLKS_MASK) | MCG_C1_CLKS(PLLFLL_SEL));
    while(PLL_SELECT_STATUS != PLLCS_SOURCE){}
}
/******************************************************************************
*   clockConfigSetSIM() - Private function to set proper clock divisions for
*   desired system level clock outputs. Also selects sources for some peripheral
*   clocks.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetSIM()
{
    /* Set divisions of 1, 3, 3, and 7 for OUTDIV1 (core/system clock), OUTDIV2
     * (bus clock), OUTDIV3 (Flexbus/SDRAMC clock), and OUTDIV4 (flash clock),
     * respectively. */
    SIM->CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(OUTDIV1_1) |
                    SIM_CLKDIV1_OUTDIV2(OUTDIV2_3) |
                    SIM_CLKDIV1_OUTDIV3(OUTDIV3_3) |
                    SIM_CLKDIV1_OUTDIV4(OUTDIV4_7));

    /* Select PLL as source for MCG PLL/FLL/IRC48M/USB1PFD clock. */
    SIM->SOPT2 = ((SIM->SOPT2 & ~SIM_SOPT2_PLLFLLSEL_MASK) |
                   SIM_SOPT2_PLLFLLSEL(MCGPLLCLK_SEL));

    /* Select RTC 32kHz as source for ERCLK32K. */
    SIM->SOPT1 = ((SIM->SOPT1 & ~SIM_SOPT1_OSC32KSEL_MASK) |
                   SIM_SOPT1_OSC32KSEL(RTC32_SEL));
}
/******************************************************************************
*   clockConfigSetCoreClock() - Private function to update project global value
*   of system core clock. This variable is used in FreeRTOSConfig.h.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetCoreClock()
{
    SystemCoreClock = CORE_CLOCK_HZ;
}
/******************************************************************************
*   clockConfigSetCLKOUTs() - Private function to set CLKOUT pins for system
*   debugging.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetCLKOUTs()
{
    /* Set CLKOUT for OSERCLK0 and RTCCLOCKOUT for RTC32K. */
    SIM->SOPT2 = ((SIM->SOPT2 & ~SIM_SOPT2_CLKOUTSEL_MASK) |
                   SIM_SOPT2_CLKOUTSEL(OSCERCLK0_SEL) |
                   SIM_SOPT2_RTCCLKOUTSEL(RTC32K_SEL));
}
/******************************************************************************
*   clockConfigSetSDHC0() - Private Function to set SDHC0 input frequency.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clockConfigSetSDHC0()
{
    /* Select core clock as input to SDHC0. */
    SIM->SOPT2 = ((SIM->SOPT2 & ~SIM_SOPT2_SDHCSRC_MASK) |
                   SIM_SOPT2_SDHCSRC(CORE_CLK_SEL));
}
