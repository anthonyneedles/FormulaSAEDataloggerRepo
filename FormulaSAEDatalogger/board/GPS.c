#include "GPS.h"
#include "MK66F18.h"
#include "FreeRTOS.h"

#define ENABLE                      1U
#define BAUD_RATE_DIVIDER       0x186U
#define BAUD_RATE_FINE_ADJUST    0x18U
#define SINGLE_STOP_BIT             0U
#define NO_PARITY_BIT               0U
#define EIGHT_BIT_MODE              0U

void GPSUART4Init()
{
    SIM->SCGC1 |= SIM_SCGC1_UART4(ENABLE);

    UART4->BDH = ((UART4->BDH & ~UART_BDH_SBR_MASK &
                   ~UART_BDH_SBNS_MASK) |
                   UART_BDH_SBNS(SINGLE_STOP_BIT) |
                   (uint8_t)(BAUD_RATE_DIVIDER >> 8));
    UART4->BDL = (uint8_t)(BAUD_RATE_DIVIDER);
    UART4->C4 = (uint8_t)(BAUD_RATE_FINE_ADJUST);
    UART4->C1 = ((UART4->C1 & ~UART_C1_PE_MASK & ~UART_C1_M_MASK) |
                  UART_C1_PE(NO_PARITY_BIT) |
                  UART_C1_M(EIGHT_BIT_MODE));
    UART4->C2 |= (UART_C2_TE(ENABLE) | UART_C2_RE(ENABLE));
}
