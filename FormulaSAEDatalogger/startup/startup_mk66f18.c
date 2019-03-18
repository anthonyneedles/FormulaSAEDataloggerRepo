//*****************************************************************************
// MK66F18 startup code for use with MCUXpresso IDE
//
// Version : 010818
//*****************************************************************************
//
// Copyright 2016-2018 NXP
//
// SPDX-License-Identifier: BSD-3-Clause
//*****************************************************************************

#if defined (DEBUG)
#pragma GCC push_options
#pragma GCC optimize ("Og")
#endif // (DEBUG)

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define WEAK_AV __attribute__ ((weak, section(".after_vectors")))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
// Flash Configuration block : 16-byte flash configuration field that stores
// default protection settings (loaded on reset) and security information that
// allows the MCU to restrict access to the Flash Memory module.
// Placed at address 0x400 by the linker script.
//*****************************************************************************
__attribute__ ((used,section(".FlashConfig"))) const struct {
    unsigned int word1;
    unsigned int word2;
    unsigned int word3;
    unsigned int word4;
} Flash_Config = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE};
//*****************************************************************************
// Declaration of external SystemInit function
//*****************************************************************************
#if defined (__USE_CMSIS)
extern void SystemInit(void);
#endif // (__USE_CMSIS)

//*****************************************************************************
// Forward declaration of the core exception handlers.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions.
// If your application is a C++ one, then any interrupt handlers defined
// in C++ files within in your main application will need to have C linkage
// rather than C++ linkage. To do this, make sure that you are using extern "C"
// { .... } around the interrupt handler within your main application code.
//*****************************************************************************
     void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

//*****************************************************************************
// Forward declaration of the application IRQ handlers. When the application
// defines a handler (with the same name), this will automatically take
// precedence over weak definitions below
//*****************************************************************************
WEAK void DMA0_DMA16_IRQHandler(void);
WEAK void DMA1_DMA17_IRQHandler(void);
WEAK void DMA2_DMA18_IRQHandler(void);
WEAK void DMA3_DMA19_IRQHandler(void);
WEAK void DMA4_DMA20_IRQHandler(void);
WEAK void DMA5_DMA21_IRQHandler(void);
WEAK void DMA6_DMA22_IRQHandler(void);
WEAK void DMA7_DMA23_IRQHandler(void);
WEAK void DMA8_DMA24_IRQHandler(void);
WEAK void DMA9_DMA25_IRQHandler(void);
WEAK void DMA10_DMA26_IRQHandler(void);
WEAK void DMA11_DMA27_IRQHandler(void);
WEAK void DMA12_DMA28_IRQHandler(void);
WEAK void DMA13_DMA29_IRQHandler(void);
WEAK void DMA14_DMA30_IRQHandler(void);
WEAK void DMA15_DMA31_IRQHandler(void);
WEAK void DMA_Error_IRQHandler(void);
WEAK void MCM_IRQHandler(void);
WEAK void FTFE_IRQHandler(void);
WEAK void Read_Collision_IRQHandler(void);
WEAK void LVD_LVW_IRQHandler(void);
WEAK void LLWU_IRQHandler(void);
WEAK void WDOG_EWM_IRQHandler(void);
WEAK void RNG_IRQHandler(void);
WEAK void I2C0_IRQHandler(void);
WEAK void I2C1_IRQHandler(void);
WEAK void SPI0_IRQHandler(void);
WEAK void SPI1_IRQHandler(void);
WEAK void I2S0_Tx_IRQHandler(void);
WEAK void I2S0_Rx_IRQHandler(void);
WEAK void Reserved46_IRQHandler(void);
WEAK void UART0_RX_TX_IRQHandler(void);
WEAK void UART0_ERR_IRQHandler(void);
WEAK void UART1_RX_TX_IRQHandler(void);
WEAK void UART1_ERR_IRQHandler(void);
WEAK void UART2_RX_TX_IRQHandler(void);
WEAK void UART2_ERR_IRQHandler(void);
WEAK void UART3_RX_TX_IRQHandler(void);
WEAK void UART3_ERR_IRQHandler(void);
WEAK void ADC0_IRQHandler(void);
WEAK void CMP0_IRQHandler(void);
WEAK void CMP1_IRQHandler(void);
WEAK void FTM0_IRQHandler(void);
WEAK void FTM1_IRQHandler(void);
WEAK void FTM2_IRQHandler(void);
WEAK void CMT_IRQHandler(void);
WEAK void RTC_IRQHandler(void);
WEAK void RTC_Seconds_IRQHandler(void);
WEAK void PIT0_IRQHandler(void);
WEAK void PIT1_IRQHandler(void);
WEAK void PIT2_IRQHandler(void);
WEAK void PIT3_IRQHandler(void);
WEAK void PDB0_IRQHandler(void);
WEAK void USB0_IRQHandler(void);
WEAK void USBDCD_IRQHandler(void);
WEAK void Reserved71_IRQHandler(void);
WEAK void DAC0_IRQHandler(void);
WEAK void MCG_IRQHandler(void);
WEAK void LPTMR0_IRQHandler(void);
WEAK void PORTA_IRQHandler(void);
WEAK void PORTB_IRQHandler(void);
WEAK void PORTC_IRQHandler(void);
WEAK void PORTD_IRQHandler(void);
WEAK void PORTE_IRQHandler(void);
WEAK void SWI_IRQHandler(void);
WEAK void SPI2_IRQHandler(void);
WEAK void UART4_RX_TX_IRQHandler(void);
WEAK void UART4_ERR_IRQHandler(void);
WEAK void Reserved84_IRQHandler(void);
WEAK void Reserved85_IRQHandler(void);
WEAK void CMP2_IRQHandler(void);
WEAK void FTM3_IRQHandler(void);
WEAK void DAC1_IRQHandler(void);
WEAK void ADC1_IRQHandler(void);
WEAK void I2C2_IRQHandler(void);
WEAK void CAN0_ORed_Message_buffer_IRQHandler(void);
WEAK void CAN0_Bus_Off_IRQHandler(void);
WEAK void CAN0_Error_IRQHandler(void);
WEAK void CAN0_Tx_Warning_IRQHandler(void);
WEAK void CAN0_Rx_Warning_IRQHandler(void);
WEAK void CAN0_Wake_Up_IRQHandler(void);
WEAK void SDHC_IRQHandler(void);
WEAK void ENET_1588_Timer_IRQHandler(void);
WEAK void ENET_Transmit_IRQHandler(void);
WEAK void ENET_Receive_IRQHandler(void);
WEAK void ENET_Error_IRQHandler(void);
WEAK void LPUART0_IRQHandler(void);
WEAK void TSI0_IRQHandler(void);
WEAK void TPM1_IRQHandler(void);
WEAK void TPM2_IRQHandler(void);
WEAK void USBHSDCD_IRQHandler(void);
WEAK void I2C3_IRQHandler(void);
WEAK void CMP3_IRQHandler(void);
WEAK void USBHS_IRQHandler(void);
WEAK void CAN1_ORed_Message_buffer_IRQHandler(void);
WEAK void CAN1_Bus_Off_IRQHandler(void);
WEAK void CAN1_Error_IRQHandler(void);
WEAK void CAN1_Tx_Warning_IRQHandler(void);
WEAK void CAN1_Rx_Warning_IRQHandler(void);
WEAK void CAN1_Wake_Up_IRQHandler(void);

//*****************************************************************************
// Forward declaration of the driver IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the driver
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//*****************************************************************************
void DMA0_DMA16_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA1_DMA17_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA2_DMA18_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA3_DMA19_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA4_DMA20_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA5_DMA21_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA6_DMA22_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA7_DMA23_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA8_DMA24_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA9_DMA25_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA10_DMA26_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA11_DMA27_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA12_DMA28_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA13_DMA29_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA14_DMA30_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA15_DMA31_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA_Error_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void MCM_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FTFE_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Read_Collision_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LVD_LVW_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LLWU_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void WDOG_EWM_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RNG_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2C0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2C1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SPI0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SPI1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2S0_Tx_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2S0_Rx_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved46_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_RX_TX_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART0_ERR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_RX_TX_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART1_ERR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_RX_TX_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART2_ERR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART3_RX_TX_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART3_ERR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ADC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FTM0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FTM1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FTM2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMT_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_Seconds_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIT0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIT1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIT2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIT3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PDB0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void USB0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void USBDCD_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved71_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DAC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void MCG_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPTMR0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PORTA_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PORTB_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PORTC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PORTD_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PORTE_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SWI_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SPI2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART4_RX_TX_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void UART4_ERR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved84_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved85_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FTM3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DAC1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ADC1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2C2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CAN0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SDHC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ENET_1588_Timer_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ENET_Transmit_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ENET_Receive_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ENET_Error_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void LPUART0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TSI0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TPM1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void TPM2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void USBHSDCD_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void I2C3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CMP3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void USBHS_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CAN1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//*****************************************************************************
#if defined (__REDLIB__)
extern void __main(void);
#endif
extern int main(void);

//*****************************************************************************
// External declaration for the pointer to the stack top from the Linker Script
//*****************************************************************************
extern void _vStackTop(void);
//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
// The vector table.
// This relies on the linker script to place at correct location in memory.
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
extern void * __Vectors __attribute__ ((alias ("g_pfnVectors")));

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM4
    &_vStackTop,                       // The initial stack pointer
    ResetISR,                          // The reset handler
    NMI_Handler,                       // The NMI handler
    HardFault_Handler,                 // The hard fault handler
    MemManage_Handler,                 // The MPU fault handler
    BusFault_Handler,                  // The bus fault handler
    UsageFault_Handler,                // The usage fault handler
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    0,                                 // Reserved
    SVC_Handler,                       // SVCall handler
    DebugMon_Handler,                  // Debug monitor handler
    0,                                 // Reserved
    PendSV_Handler,                    // The PendSV handler
    SysTick_Handler,                   // The SysTick handler

    // Chip Level - MK66F18
    DMA0_DMA16_IRQHandler,                // 16 : DMA Channel 0, 16 Transfer Complete
    DMA1_DMA17_IRQHandler,                // 17 : DMA Channel 1, 17 Transfer Complete
    DMA2_DMA18_IRQHandler,                // 18 : DMA Channel 2, 18 Transfer Complete
    DMA3_DMA19_IRQHandler,                // 19 : DMA Channel 3, 19 Transfer Complete
    DMA4_DMA20_IRQHandler,                // 20 : DMA Channel 4, 20 Transfer Complete
    DMA5_DMA21_IRQHandler,                // 21 : DMA Channel 5, 21 Transfer Complete
    DMA6_DMA22_IRQHandler,                // 22 : DMA Channel 6, 22 Transfer Complete
    DMA7_DMA23_IRQHandler,                // 23 : DMA Channel 7, 23 Transfer Complete
    DMA8_DMA24_IRQHandler,                // 24 : DMA Channel 8, 24 Transfer Complete
    DMA9_DMA25_IRQHandler,                // 25 : DMA Channel 9, 25 Transfer Complete
    DMA10_DMA26_IRQHandler,               // 26 : DMA Channel 10, 26 Transfer Complete
    DMA11_DMA27_IRQHandler,               // 27 : DMA Channel 11, 27 Transfer Complete
    DMA12_DMA28_IRQHandler,               // 28 : DMA Channel 12, 28 Transfer Complete
    DMA13_DMA29_IRQHandler,               // 29 : DMA Channel 13, 29 Transfer Complete
    DMA14_DMA30_IRQHandler,               // 30 : DMA Channel 14, 30 Transfer Complete
    DMA15_DMA31_IRQHandler,               // 31 : DMA Channel 15, 31 Transfer Complete
    DMA_Error_IRQHandler,                 // 32 : DMA Error Interrupt
    MCM_IRQHandler,                       // 33 : Normal Interrupt
    FTFE_IRQHandler,                      // 34 : FTFE Command complete interrupt
    Read_Collision_IRQHandler,            // 35 : Read Collision Interrupt
    LVD_LVW_IRQHandler,                   // 36 : Low Voltage Detect, Low Voltage Warning
    LLWU_IRQHandler,                      // 37 : Low Leakage Wakeup Unit
    WDOG_EWM_IRQHandler,                  // 38 : WDOG Interrupt
    RNG_IRQHandler,                       // 39 : RNG Interrupt
    I2C0_IRQHandler,                      // 40 : I2C0 interrupt
    I2C1_IRQHandler,                      // 41 : I2C1 interrupt
    SPI0_IRQHandler,                      // 42 : SPI0 Interrupt
    SPI1_IRQHandler,                      // 43 : SPI1 Interrupt
    I2S0_Tx_IRQHandler,                   // 44 : I2S0 transmit interrupt
    I2S0_Rx_IRQHandler,                   // 45 : I2S0 receive interrupt
    Reserved46_IRQHandler,                // 46 : Reserved interrupt
    UART0_RX_TX_IRQHandler,               // 47 : UART0 Receive/Transmit interrupt
    UART0_ERR_IRQHandler,                 // 48 : UART0 Error interrupt
    UART1_RX_TX_IRQHandler,               // 49 : UART1 Receive/Transmit interrupt
    UART1_ERR_IRQHandler,                 // 50 : UART1 Error interrupt
    UART2_RX_TX_IRQHandler,               // 51 : UART2 Receive/Transmit interrupt
    UART2_ERR_IRQHandler,                 // 52 : UART2 Error interrupt
    UART3_RX_TX_IRQHandler,               // 53 : UART3 Receive/Transmit interrupt
    UART3_ERR_IRQHandler,                 // 54 : UART3 Error interrupt
    ADC0_IRQHandler,                      // 55 : ADC0 interrupt
    CMP0_IRQHandler,                      // 56 : CMP0 interrupt
    CMP1_IRQHandler,                      // 57 : CMP1 interrupt
    FTM0_IRQHandler,                      // 58 : FTM0 fault, overflow and channels interrupt
    FTM1_IRQHandler,                      // 59 : FTM1 fault, overflow and channels interrupt
    FTM2_IRQHandler,                      // 60 : FTM2 fault, overflow and channels interrupt
    CMT_IRQHandler,                       // 61 : CMT interrupt
    RTC_IRQHandler,                       // 62 : RTC interrupt
    RTC_Seconds_IRQHandler,               // 63 : RTC seconds interrupt
    PIT0_IRQHandler,                      // 64 : PIT timer channel 0 interrupt
    PIT1_IRQHandler,                      // 65 : PIT timer channel 1 interrupt
    PIT2_IRQHandler,                      // 66 : PIT timer channel 2 interrupt
    PIT3_IRQHandler,                      // 67 : PIT timer channel 3 interrupt
    PDB0_IRQHandler,                      // 68 : PDB0 Interrupt
    USB0_IRQHandler,                      // 69 : USB0 interrupt
    USBDCD_IRQHandler,                    // 70 : USBDCD Interrupt
    Reserved71_IRQHandler,                // 71 : Reserved interrupt
    DAC0_IRQHandler,                      // 72 : DAC0 interrupt
    MCG_IRQHandler,                       // 73 : MCG Interrupt
    LPTMR0_IRQHandler,                    // 74 : LPTimer interrupt
    PORTA_IRQHandler,                     // 75 : Port A interrupt
    PORTB_IRQHandler,                     // 76 : Port B interrupt
    PORTC_IRQHandler,                     // 77 : Port C interrupt
    PORTD_IRQHandler,                     // 78 : Port D interrupt
    PORTE_IRQHandler,                     // 79 : Port E interrupt
    SWI_IRQHandler,                       // 80 : Software interrupt
    SPI2_IRQHandler,                      // 81 : SPI2 Interrupt
    UART4_RX_TX_IRQHandler,               // 82 : UART4 Receive/Transmit interrupt
    UART4_ERR_IRQHandler,                 // 83 : UART4 Error interrupt
    Reserved84_IRQHandler,                // 84 : Reserved interrupt
    Reserved85_IRQHandler,                // 85 : Reserved interrupt
    CMP2_IRQHandler,                      // 86 : CMP2 interrupt
    FTM3_IRQHandler,                      // 87 : FTM3 fault, overflow and channels interrupt
    DAC1_IRQHandler,                      // 88 : DAC1 interrupt
    ADC1_IRQHandler,                      // 89 : ADC1 interrupt
    I2C2_IRQHandler,                      // 90 : I2C2 interrupt
    CAN0_ORed_Message_buffer_IRQHandler,  // 91 : CAN0 OR'd message buffers interrupt
    CAN0_Bus_Off_IRQHandler,              // 92 : CAN0 bus off interrupt
    CAN0_Error_IRQHandler,                // 93 : CAN0 error interrupt
    CAN0_Tx_Warning_IRQHandler,           // 94 : CAN0 Tx warning interrupt
    CAN0_Rx_Warning_IRQHandler,           // 95 : CAN0 Rx warning interrupt
    CAN0_Wake_Up_IRQHandler,              // 96 : CAN0 wake up interrupt
    SDHC_IRQHandler,                      // 97 : SDHC interrupt
    ENET_1588_Timer_IRQHandler,           // 98 : Ethernet MAC IEEE 1588 Timer Interrupt
    ENET_Transmit_IRQHandler,             // 99 : Ethernet MAC Transmit Interrupt
    ENET_Receive_IRQHandler,              // 100: Ethernet MAC Receive Interrupt
    ENET_Error_IRQHandler,                // 101: Ethernet MAC Error and miscelaneous Interrupt
    LPUART0_IRQHandler,                   // 102: LPUART0 status/error interrupt
    TSI0_IRQHandler,                      // 103: TSI0 interrupt
    TPM1_IRQHandler,                      // 104: TPM1 fault, overflow and channels interrupt
    TPM2_IRQHandler,                      // 105: TPM2 fault, overflow and channels interrupt
    USBHSDCD_IRQHandler,                  // 106: USBHSDCD, USBHS Phy Interrupt
    I2C3_IRQHandler,                      // 107: I2C3 interrupt
    CMP3_IRQHandler,                      // 108: CMP3 interrupt
    USBHS_IRQHandler,                     // 109: USB high speed OTG interrupt
    CAN1_ORed_Message_buffer_IRQHandler,  // 110: CAN1 OR'd message buffers interrupt
    CAN1_Bus_Off_IRQHandler,              // 111: CAN1 bus off interrupt
    CAN1_Error_IRQHandler,                // 112: CAN1 error interrupt
    CAN1_Tx_Warning_IRQHandler,           // 113: CAN1 Tx warning interrupt
    CAN1_Rx_Warning_IRQHandler,           // 114: CAN1 Rx warning interrupt
    CAN1_Wake_Up_IRQHandler,              // 115: CAN1 wake up interrupt

}; /* End of g_pfnVectors */

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors.init_data")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int *pulSrc = (unsigned int*) romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors.init_bss")))
void bss_init(unsigned int start, unsigned int len) {
    unsigned int *pulDest = (unsigned int*) start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors.reset")))
void ResetISR(void) {

    // Disable interrupts
    __asm volatile ("cpsid i");

#if defined (__USE_CMSIS)
// If __USE_CMSIS defined, then call CMSIS SystemInit code
    SystemInit();

#else
    // Disable Watchdog
    //  Write 0xC520 to watchdog unlock register
    *((volatile unsigned short *)0x4005200E) = 0xC520;
    //  Followed by 0xD928 to complete the unlock
    *((volatile unsigned short *)0x4005200E) = 0xD928;
    // Now disable watchdog via STCTRLH register
    *((volatile unsigned short *)0x40052000) = 0x01D2u;
#endif // (__USE_CMSIS)

    //
    // Copy the data sections from flash to SRAM.
    //
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int *SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end) {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }

    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end) {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }

#if !defined (__USE_CMSIS)
// Assume that if __USE_CMSIS defined, then CMSIS SystemInit code
// will enable the FPU
#if defined (__VFP_FP__) && !defined (__SOFTFP__)
    //
    // Code to enable the Cortex-M4 FPU only included
    // if appropriate build options have been selected.
    // Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C)
    //
    // Read CPACR (located at address 0xE000ED88)
    // Set bits 20-23 to enable CP10 and CP11 coprocessors
    // Write back the modified value to the CPACR
    asm volatile ("LDR.W R0, =0xE000ED88\n\t"
                  "LDR R1, [R0]\n\t"
                  "ORR R1, R1, #(0xF << 20)\n\t"
                  "STR R1, [R0]");
#endif // (__VFP_FP__) && !(__SOFTFP__)
#endif // (__USE_CMSIS)

#if !defined (__USE_CMSIS)
// Assume that if __USE_CMSIS defined, then CMSIS SystemInit code
// will setup the VTOR register

    // Check to see if we are running the code from a non-zero
    // address (eg RAM, external flash), in which case we need
    // to modify the VTOR register to tell the CPU that the
    // vector table is located at a non-0x0 address.
    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    if ((unsigned int *)g_pfnVectors!=(unsigned int *) 0x00000000) {
        *pSCB_VTOR = (unsigned int)g_pfnVectors;
    }
#endif // (__USE_CMSIS)

#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif

    // Reenable interrupts
    __asm volatile ("cpsie i");

#if defined (__REDLIB__)
    // Call the Redlib library, which in turn calls main()
    __main();
#else
    main();
#endif

    //
    // main() shouldn't return, but if it does, we'll just enter an infinite loop
    //
    while (1) {
        ;
    }
}

//*****************************************************************************
// Default core exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
WEAK_AV void NMI_Handler(void)
{ while(1) {}
}

WEAK_AV void HardFault_Handler(void)
{ while(1) {}
}

WEAK_AV void MemManage_Handler(void)
{ while(1) {}
}

WEAK_AV void BusFault_Handler(void)
{ while(1) {}
}

WEAK_AV void UsageFault_Handler(void)
{ while(1) {}
}

WEAK_AV void SVC_Handler(void)
{ while(1) {}
}

WEAK_AV void DebugMon_Handler(void)
{ while(1) {}
}

WEAK_AV void PendSV_Handler(void)
{ while(1) {}
}

WEAK_AV void SysTick_Handler(void)
{ while(1) {}
}

//*****************************************************************************
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//*****************************************************************************
WEAK_AV void IntDefaultHandler(void)
{ while(1) {}
}

//*****************************************************************************
// Default application exception handlers. Override the ones here by defining
// your own handler routines in your application code. These routines call
// driver exception handlers or IntDefaultHandler() if no driver exception
// handler is included.
//*****************************************************************************
WEAK void DMA0_DMA16_IRQHandler(void)
{   DMA0_DMA16_DriverIRQHandler();
}

WEAK void DMA1_DMA17_IRQHandler(void)
{   DMA1_DMA17_DriverIRQHandler();
}

WEAK void DMA2_DMA18_IRQHandler(void)
{   DMA2_DMA18_DriverIRQHandler();
}

WEAK void DMA3_DMA19_IRQHandler(void)
{   DMA3_DMA19_DriverIRQHandler();
}

WEAK void DMA4_DMA20_IRQHandler(void)
{   DMA4_DMA20_DriverIRQHandler();
}

WEAK void DMA5_DMA21_IRQHandler(void)
{   DMA5_DMA21_DriverIRQHandler();
}

WEAK void DMA6_DMA22_IRQHandler(void)
{   DMA6_DMA22_DriverIRQHandler();
}

WEAK void DMA7_DMA23_IRQHandler(void)
{   DMA7_DMA23_DriverIRQHandler();
}

WEAK void DMA8_DMA24_IRQHandler(void)
{   DMA8_DMA24_DriverIRQHandler();
}

WEAK void DMA9_DMA25_IRQHandler(void)
{   DMA9_DMA25_DriverIRQHandler();
}

WEAK void DMA10_DMA26_IRQHandler(void)
{   DMA10_DMA26_DriverIRQHandler();
}

WEAK void DMA11_DMA27_IRQHandler(void)
{   DMA11_DMA27_DriverIRQHandler();
}

WEAK void DMA12_DMA28_IRQHandler(void)
{   DMA12_DMA28_DriverIRQHandler();
}

WEAK void DMA13_DMA29_IRQHandler(void)
{   DMA13_DMA29_DriverIRQHandler();
}

WEAK void DMA14_DMA30_IRQHandler(void)
{   DMA14_DMA30_DriverIRQHandler();
}

WEAK void DMA15_DMA31_IRQHandler(void)
{   DMA15_DMA31_DriverIRQHandler();
}

WEAK void DMA_Error_IRQHandler(void)
{   DMA_Error_DriverIRQHandler();
}

WEAK void MCM_IRQHandler(void)
{   MCM_DriverIRQHandler();
}

WEAK void FTFE_IRQHandler(void)
{   FTFE_DriverIRQHandler();
}

WEAK void Read_Collision_IRQHandler(void)
{   Read_Collision_DriverIRQHandler();
}

WEAK void LVD_LVW_IRQHandler(void)
{   LVD_LVW_DriverIRQHandler();
}

WEAK void LLWU_IRQHandler(void)
{   LLWU_DriverIRQHandler();
}

WEAK void WDOG_EWM_IRQHandler(void)
{   WDOG_EWM_DriverIRQHandler();
}

WEAK void RNG_IRQHandler(void)
{   RNG_DriverIRQHandler();
}

WEAK void I2C0_IRQHandler(void)
{   I2C0_DriverIRQHandler();
}

WEAK void I2C1_IRQHandler(void)
{   I2C1_DriverIRQHandler();
}

WEAK void SPI0_IRQHandler(void)
{   SPI0_DriverIRQHandler();
}

WEAK void SPI1_IRQHandler(void)
{   SPI1_DriverIRQHandler();
}

WEAK void I2S0_Tx_IRQHandler(void)
{   I2S0_Tx_DriverIRQHandler();
}

WEAK void I2S0_Rx_IRQHandler(void)
{   I2S0_Rx_DriverIRQHandler();
}

WEAK void Reserved46_IRQHandler(void)
{   Reserved46_DriverIRQHandler();
}

WEAK void UART0_RX_TX_IRQHandler(void)
{   UART0_RX_TX_DriverIRQHandler();
}

WEAK void UART0_ERR_IRQHandler(void)
{   UART0_ERR_DriverIRQHandler();
}

WEAK void UART1_RX_TX_IRQHandler(void)
{   UART1_RX_TX_DriverIRQHandler();
}

WEAK void UART1_ERR_IRQHandler(void)
{   UART1_ERR_DriverIRQHandler();
}

WEAK void UART2_RX_TX_IRQHandler(void)
{   UART2_RX_TX_DriverIRQHandler();
}

WEAK void UART2_ERR_IRQHandler(void)
{   UART2_ERR_DriverIRQHandler();
}

WEAK void UART3_RX_TX_IRQHandler(void)
{   UART3_RX_TX_DriverIRQHandler();
}

WEAK void UART3_ERR_IRQHandler(void)
{   UART3_ERR_DriverIRQHandler();
}

WEAK void ADC0_IRQHandler(void)
{   ADC0_DriverIRQHandler();
}

WEAK void CMP0_IRQHandler(void)
{   CMP0_DriverIRQHandler();
}

WEAK void CMP1_IRQHandler(void)
{   CMP1_DriverIRQHandler();
}

WEAK void FTM0_IRQHandler(void)
{   FTM0_DriverIRQHandler();
}

WEAK void FTM1_IRQHandler(void)
{   FTM1_DriverIRQHandler();
}

WEAK void FTM2_IRQHandler(void)
{   FTM2_DriverIRQHandler();
}

WEAK void CMT_IRQHandler(void)
{   CMT_DriverIRQHandler();
}

WEAK void RTC_IRQHandler(void)
{   RTC_DriverIRQHandler();
}

WEAK void RTC_Seconds_IRQHandler(void)
{   RTC_Seconds_DriverIRQHandler();
}

WEAK void PIT0_IRQHandler(void)
{   PIT0_DriverIRQHandler();
}

WEAK void PIT1_IRQHandler(void)
{   PIT1_DriverIRQHandler();
}

WEAK void PIT2_IRQHandler(void)
{   PIT2_DriverIRQHandler();
}

WEAK void PIT3_IRQHandler(void)
{   PIT3_DriverIRQHandler();
}

WEAK void PDB0_IRQHandler(void)
{   PDB0_DriverIRQHandler();
}

WEAK void USB0_IRQHandler(void)
{   USB0_DriverIRQHandler();
}

WEAK void USBDCD_IRQHandler(void)
{   USBDCD_DriverIRQHandler();
}

WEAK void Reserved71_IRQHandler(void)
{   Reserved71_DriverIRQHandler();
}

WEAK void DAC0_IRQHandler(void)
{   DAC0_DriverIRQHandler();
}

WEAK void MCG_IRQHandler(void)
{   MCG_DriverIRQHandler();
}

WEAK void LPTMR0_IRQHandler(void)
{   LPTMR0_DriverIRQHandler();
}

WEAK void PORTA_IRQHandler(void)
{   PORTA_DriverIRQHandler();
}

WEAK void PORTB_IRQHandler(void)
{   PORTB_DriverIRQHandler();
}

WEAK void PORTC_IRQHandler(void)
{   PORTC_DriverIRQHandler();
}

WEAK void PORTD_IRQHandler(void)
{   PORTD_DriverIRQHandler();
}

WEAK void PORTE_IRQHandler(void)
{   PORTE_DriverIRQHandler();
}

WEAK void SWI_IRQHandler(void)
{   SWI_DriverIRQHandler();
}

WEAK void SPI2_IRQHandler(void)
{   SPI2_DriverIRQHandler();
}

WEAK void UART4_RX_TX_IRQHandler(void)
{   UART4_RX_TX_DriverIRQHandler();
}

WEAK void UART4_ERR_IRQHandler(void)
{   UART4_ERR_DriverIRQHandler();
}

WEAK void Reserved84_IRQHandler(void)
{   Reserved84_DriverIRQHandler();
}

WEAK void Reserved85_IRQHandler(void)
{   Reserved85_DriverIRQHandler();
}

WEAK void CMP2_IRQHandler(void)
{   CMP2_DriverIRQHandler();
}

WEAK void FTM3_IRQHandler(void)
{   FTM3_DriverIRQHandler();
}

WEAK void DAC1_IRQHandler(void)
{   DAC1_DriverIRQHandler();
}

WEAK void ADC1_IRQHandler(void)
{   ADC1_DriverIRQHandler();
}

WEAK void I2C2_IRQHandler(void)
{   I2C2_DriverIRQHandler();
}

WEAK void CAN0_ORed_Message_buffer_IRQHandler(void)
{   CAN0_DriverIRQHandler();
}

WEAK void CAN0_Bus_Off_IRQHandler(void)
{   CAN0_DriverIRQHandler();
}

WEAK void CAN0_Error_IRQHandler(void)
{   CAN0_DriverIRQHandler();
}

WEAK void CAN0_Tx_Warning_IRQHandler(void)
{   CAN0_DriverIRQHandler();
}

WEAK void CAN0_Rx_Warning_IRQHandler(void)
{   CAN0_DriverIRQHandler();
}

WEAK void CAN0_Wake_Up_IRQHandler(void)
{   CAN0_DriverIRQHandler();
}

WEAK void SDHC_IRQHandler(void)
{   SDHC_DriverIRQHandler();
}

WEAK void ENET_1588_Timer_IRQHandler(void)
{   ENET_1588_Timer_DriverIRQHandler();
}

WEAK void ENET_Transmit_IRQHandler(void)
{   ENET_Transmit_DriverIRQHandler();
}

WEAK void ENET_Receive_IRQHandler(void)
{   ENET_Receive_DriverIRQHandler();
}

WEAK void ENET_Error_IRQHandler(void)
{   ENET_Error_DriverIRQHandler();
}

WEAK void LPUART0_IRQHandler(void)
{   LPUART0_DriverIRQHandler();
}

WEAK void TSI0_IRQHandler(void)
{   TSI0_DriverIRQHandler();
}

WEAK void TPM1_IRQHandler(void)
{   TPM1_DriverIRQHandler();
}

WEAK void TPM2_IRQHandler(void)
{   TPM2_DriverIRQHandler();
}

WEAK void USBHSDCD_IRQHandler(void)
{   USBHSDCD_DriverIRQHandler();
}

WEAK void I2C3_IRQHandler(void)
{   I2C3_DriverIRQHandler();
}

WEAK void CMP3_IRQHandler(void)
{   CMP3_DriverIRQHandler();
}

WEAK void USBHS_IRQHandler(void)
{   USBHS_DriverIRQHandler();
}

WEAK void CAN1_ORed_Message_buffer_IRQHandler(void)
{   CAN1_DriverIRQHandler();
}

WEAK void CAN1_Bus_Off_IRQHandler(void)
{   CAN1_DriverIRQHandler();
}

WEAK void CAN1_Error_IRQHandler(void)
{   CAN1_DriverIRQHandler();
}

WEAK void CAN1_Tx_Warning_IRQHandler(void)
{   CAN1_DriverIRQHandler();
}

WEAK void CAN1_Rx_Warning_IRQHandler(void)
{   CAN1_DriverIRQHandler();
}

WEAK void CAN1_Wake_Up_IRQHandler(void)
{   CAN1_DriverIRQHandler();
}

//*****************************************************************************

#if defined (DEBUG)
#pragma GCC pop_options
#endif // (DEBUG)
