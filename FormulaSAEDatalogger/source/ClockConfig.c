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
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/01/2019
*
*   Created on: 03/12/2019
*   Author: Anthony Needles
******************************************************************************/
#include "MK66F18.h"
#include "ClockConfig.h"
/******************************************************************************
*   Private function prototypes
******************************************************************************/
static void clkcfgSetHSRun(void);
static void clkcfgSetXtalRTC(void);
static void clkcfgSetXtalOSC(void);
static void clkcfgSetMCG(void);
static void clkcfgSetCoreClock(void);
static void clkcfgSetCLKOUTs(void);
static void clkcfgSetSDHC0(void);
/******************************************************************************
*   Private Definitions
******************************************************************************/
#define ENABLE                      0x01U
#define DISABLE                     0x00U
#define HSRUN                       0x03U
#define PWRMODE_HSRUN               0x80U
#define SAFE_DIVISIONS        0x02260000U
#define CRYSTL_REF                  0x01U
#define LOW_POWER                   0x00U
#define VH_FREQ_RANGE               0x02U
#define EXT_REF_CLK_MODE            0x02U
#define FRDRIV_256                  0x03U
#define OSC_READY                   0x01U
#define EXT_REF                     0x00U
#define VMLTPY_30                   0x0EU
#define PLL_CLK_SEL                 0x01U
#define PLL_CLOCK                   0x01U
#define PLL_LOCKED                  0x01U
#define CORE_CLOCK_HZ          180000000U
#define CORE_CLK_DIV_1              0x00U
#define BUS_CLK_DIV_3               0x02U
#define FLEXBUS_CLK_DIV_3           0x02U
#define FLASH_CLK_DIV_6             0x06U
#define PLLFLL_SEL                  0x00U
#define PLL_OUT_MODE                0x03U
#define RTC_1HZ                     0x00U
#define ALT_5_CLKOUT                0x05U
#define ALT_6_RTCCLKOUT             0x06U
#define CORE_CLK_SEL                0x00U
#define RTC_1_HZ                    0x00U
#define FLEXBUS_CLKOUT              0x00U

#define POWER_MODE_STATUS   (SMC->PMSTAT)
#define OSC_STATUS          ((MCG->S & MCG_S_OSCINIT0_MASK) >> MCG_S_OSCINIT0_SHIFT)
#define FLL_REF_STATUS      ((MCG->S & MCG_S_IREFST_MASK) >> MCG_S_IREFST_SHIFT)
#define PLLS_SOURCE_STATUS  ((MCG->S & MCG_S_PLLST_MASK) >> MCG_S_PLLST_SHIFT)
#define PLL_LOCK_STATUS     ((MCG->S & MCG_S_LOCK0_MASK) >> MCG_S_LOCK0_SHIFT)
#define CLK_MODE_STATUS     ((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT)
/*******************************************************************************
 * Variables
 ******************************************************************************/
extern uint32_t SystemCoreClock;
/******************************************************************************
*   ClkCfgRun() - Public function to run all private clock configuration
*   functions in valid order.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
void ClkCfgRun()
{
    clkcfgSetHSRun();
    clkcfgSetXtalOSC();
    clkcfgSetXtalRTC();
    clkcfgSetMCG();
    clkcfgSetCoreClock();
    clkcfgSetCLKOUTs();
    clkcfgSetSDHC0();
}
/******************************************************************************
*   clkcfgSetHSRun() - Private function to enable system for HSRUN mode.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetHSRun()
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
*   clkcfgSetXtalRTC() - Private function to set RTC module for peripheral
*   output and enabling 32k oscillator. Must wait 1000ms before enabling time
*   counter to ensure oscillator stabilization.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetXtalRTC()
{
    /* Allow access and interrupts for RTC. */
    SIM->SCGC6 |= SIM_SCGC6_RTC(ENABLE);
    /* Set 32k clock output to peripherals and enable 32k oscillator. Internal
     * 4pF load enabled. */
    RTC->CR = ((RTC_CR_OSCE(ENABLE) | RTC_CR_SC4P(ENABLE)) & ~RTC_CR_CLKO_MASK);

    /* Time invalid due to software reset. Time count is disabled, TSR is
     * written to, then finally time count is enabled.
     * NOTE: Must be 1000ms gap between enabling oscillator and enabling TCE. */
    RTC->SR &= ~RTC_SR_TCE_MASK;
    RTC->TSR = 1;
    RTC->SR |= RTC_SR_TCE_MASK;

    /* Disallow access and interrupts for RTC. */
    SIM->SCGC6 &= ~SIM_SCGC6_RTC(ENABLE);
}
/******************************************************************************
*   clkcfgSetXtalOSC() - Private function to enable external reference
*   clock in Connection 3 (external capacitors, internal feedback resistor).
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetXtalOSC()
{
    /* Enables OSC to enable external reference clock. */
    OSC->CR |= OSC_CR_ERCLKEN(ENABLE);
}
/******************************************************************************
*   clkcfgSetMCG() - Private function to configure the MCG for desired
*   operation of PEE (PLL Engaged External). An external source (external
*   crystal & system oscillator) engaged to the PLL supplying MCGOUTCLK. This
*   will achieve a clock value of 180MHz on MCGOUTCLK to provide the SIM.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetMCG()
{
    /* MCG comes out of reset in FEI (FLL Engaged Internal) mode. The desired
     * mode is PEE. The MCG must first transition from FEI to FBE (FLL Bypass
     * External), then transition from FBE to PBE (PLL Bypass External), and
     * finally from PBE to PEE. */

    /* Transition to FBE: */

    /* Config MCG for very high frequency range (due to 12MHz external crystal),
     * low power operation, and a crystal external reference. */
    MCG->C2 = (MCG_C2_EREFS(CRYSTL_REF) | MCG_C2_HGO(LOW_POWER) |
               MCG_C2_RANGE(VH_FREQ_RANGE));
    /* MCGOUTCLOCK will (temporarily) output external reference clock (12MHz).
     * FLL external reference divider will 256 as the 31.25kHz to 39.0625kHz
     * FLL input range is only required to be met when the MCGOUTCLK source
     * is FLL, but the closest value will be reached. */
    MCG->C1 = (MCG_C1_CLKS(EXT_REF_CLK_MODE) | MCG_C1_FRDIV(FRDRIV_256));
    /* Wait until crystal oscillator stabilizes. */
    while(OSC_STATUS != OSC_READY){}
    /* Wait until source of FLL reference clock is external reference clock
     * (this does not immediately happen upon clearing IREFS in C2). */
    while(FLL_REF_STATUS != EXT_REF){}
    /* Wait until clock mode status is external reference clock
     * (this does not immediately happen upon setting CLKS in C1). */
    while(CLK_MODE_STATUS != EXT_REF_CLK_MODE){}

    /* Transition to PBE: */

    /* PLL will multiply by 30 to produce a MCGPLL0CLK2X frequency of 12MHz*30 =
     * 360MHz. This will then be divided by 2 to achieve MCGPLL0CLK, 180MHZ.
     * The PLL is chosen as input to PLLS for later use as MCGOUTCLK. */
    MCG->C6 = (MCG_C6_PLLS(PLL_CLK_SEL) | MCG_C6_VDIV(VMLTPY_30));
    /* Wait until PLLS source of PLL synchronization. */
    while(PLLS_SOURCE_STATUS != PLL_CLOCK){}
    /* Wait until PLL has obtained lock and MCGPLLCLK is no longer gated off */
    while(PLL_LOCK_STATUS != PLL_LOCKED){}

    /*  Set proper clock divisions for desired system level clock outputs. */
    SIM->CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(CORE_CLK_DIV_1) |
                    SIM_CLKDIV1_OUTDIV2(BUS_CLK_DIV_3) |
                    SIM_CLKDIV1_OUTDIV3(FLEXBUS_CLK_DIV_3) |
                    SIM_CLKDIV1_OUTDIV4(FLASH_CLK_DIV_6));

    /* Transition to PEE: */

    /* Configure MCGOUTCLK for source of FLL/PLLCS (in this case 180MHz PLL). */
    MCG->C1 = ((MCG->C1 & ~MCG_C1_CLKS_MASK) | MCG_C1_CLKS(PLLFLL_SEL));
    /* Wait for PLL clock mode to update to PLL output.
     * (this does not immediately happen upon setting CLKST in C1) */
    while(CLK_MODE_STATUS != PLL_OUT_MODE){}
}
/******************************************************************************
*   clkcfgSetCoreClock() - Private function to update project global value
*   of system core clock. This variable is used in FreeRTOSConfig.h.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetCoreClock()
{
    SystemCoreClock = CORE_CLOCK_HZ;
}
/******************************************************************************
*   clkcfgSetCLKOUTs() - Private function to set CLKOUT pins for system
*   debugging.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetCLKOUTs()
{
    /* Set CLKOUT (PORT C PIN 3) for FlexBus CLKOUT and RTCCLOCKOUT (PORT E
     * PIN 26) for RTC32K. */
    SIM->SOPT2 |= ((SIM->SOPT2 & ~SIM_SOPT2_CLKOUTSEL_MASK) |
                    SIM_SOPT2_CLKOUTSEL(FLEXBUS_CLKOUT) |
                    SIM_SOPT2_RTCCLKOUTSEL(RTC_1_HZ));
    /* Enable CLKOUT and RTC_CLKOUT ports and mux functionality as needed */
    SIM->SCGC5 |= (SIM_SCGC5_PORTC(ENABLE) | SIM_SCGC5_PORTE(ENABLE));
    PORTC->PCR[3] = ((PORTC->PCR[3] & ~PORT_PCR_MUX_MASK) |
                      PORT_PCR_MUX(ALT_5_CLKOUT));
    PORTE->PCR[26] = ((PORTE->PCR[26] & ~PORT_PCR_MUX_MASK) |
                       PORT_PCR_MUX(ALT_6_RTCCLKOUT));
}
/******************************************************************************
*   clkcfgSetSDHC0() - Private Function to set SDHC0 input frequency.
*
*   Parameters: None
*
*   Return: None
******************************************************************************/
static void clkcfgSetSDHC0()
{
    /* Select core clock as input to SDHC0. */
    SIM->SOPT2 = ((SIM->SOPT2 & ~SIM_SOPT2_SDHCSRC_MASK) |
                   SIM_SOPT2_SDHCSRC(CORE_CLK_SEL));
}
