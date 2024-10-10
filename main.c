#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

uint8_t receivedByte;
bool dataReceivedFlag = true;

void UART5_send(void);
void UART5_Transmit(uint8_t data);
