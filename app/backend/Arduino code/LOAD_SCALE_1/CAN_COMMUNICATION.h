#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include <SPI.h>
#include <mcp_can.h>

// CAN module CS pin
const int SPI_CS_PIN = 10;

extern MCP_CAN CAN;// Set CS pin

// Timeout value for waiting for a response (milliseconds)
const unsigned long TIMEOUT_MS = 1000;

void initCAN();
bool sendCANMessage(unsigned long id, unsigned char *msg, unsigned char len);
bool receiveCANMessage(unsigned long &id, unsigned char *buf, unsigned char &len);

#endif
