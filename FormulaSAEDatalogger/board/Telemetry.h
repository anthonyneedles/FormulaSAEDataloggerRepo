/******************************************************************************
*   Telemetry.c
*
*   See /FormulaSAEDatalogger/README.txt for detailed changelog.
*
*   https://github.com/anthonyneedles/FormulaSAEDataloggerRepo
*
*   The Telemetry module of the Formula SAE Datalogger initializes UART3 for
*   communication to and from the wireless telemetry unit. At 2Hz, a package of
*   requested data is sent, along with a time stamp. In addition, at 2Hz,
*   configuration data is received by the wireless telemetry unit.
*
*   MCU: MK66FN2M0VLQ18R
*
*   Comments up to date as of: 05/29/2019
*
*   Created on: 05/20/2019
*   Author: Anthony Needles
******************************************************************************/
#ifndef TELEMETRY_H_
#define TELEMETRY_H_

/******************************************************************************
*   Public Function Prototypes
******************************************************************************/
/******************************************************************************
*   TelInit() - Public function to configure UART3 for reception and
*   transmission with the separate wireless telemetry unit.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void TelInit(void);

/******************************************************************************
*   UART3_RX_TX_IRQHandler() - Interrupt handler for UART3 TDRE and RDRF flags.
*   This triggers upon both empty transmit buffer and full receive buffer.
*   Clears TDRE flag by reading UART3 S1 register and writing to the UART3 D
*   register. Clears RDRF by reading UART3 S1 register and reading from the
*   UART3 D register. Posts task notification to corresponding Telemetry
*   Input/Output Task. This allows non-blocking UART driver operation.
*
*   Parameters: None
*
*   Return: None
*
*   Author: Anthony Needles
******************************************************************************/
void UART3_RX_TX_IRQHandler(void);

#endif /* TELEMETRY_H_ */
