#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

uint8_t receivedByte;
bool dataReceivedFlag = true;

void UART5_send(void);
void UART5_Transmit(uint8_t data);

void PortF_Initialisation(void){

    // PORTF, PF7-PF0, PF4-0X01, PF3-green, PF2-blue, PF1-red, PF0-0X10
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;   // Enable clock for Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port F
    GPIO_PORTF_CR_R = 0x1f;                 // Commit changes,1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DEN_R = 0x1f;                // Digital function enable, 1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DIR_R = 0x0e;                // Set output/input, 1-output (PF7-PF0 = 00001110)
    GPIO_PORTF_PUR_R = 0x11;                // Enable pull-up resistor, 1-enable (PF7-PF0 = 00010001)
    GPIO_PORTF_DATA_R = 0x00;               // Reset the data register (PF7-PF0 = 00000000)

}
