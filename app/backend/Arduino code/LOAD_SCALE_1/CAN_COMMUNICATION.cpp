#include "CAN_COMMUNICATION.h"

MCP_CAN CAN(SPI_CS_PIN);

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Initializes the CAN bus communication.
//              This function should be called once during setup to initialize the MCP_CAN module.
// ARGUMENTS:   None
// RETURN VALUE: None
void initCAN() {

  // Initialize MCP_CAN module
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("CAN BUS Shield init ok!");
  } else {
    Serial.println("CAN BUS Shield init failed");
    while (1); // Halt if initialization failed
  }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Sends a CAN message with the specified ID, data, and length.
// ARGUMENTS:
//   id: The identifier of the CAN message.
//   msg: A pointer to the array containing the data bytes to be sent.
//   len: The length of the data in bytes.
// RETURN VALUE:
//   true if the message was sent successfully, false otherwise.
bool sendCANMessage(unsigned long id, unsigned char *msg, unsigned char len) {
  
  // Attempt to send the message
  int result = CAN.sendMsgBuf(id, 0, len, msg);
  if (result == CAN_OK) {
    return true; // Message sent successfully
  } else {
    Serial.print("Failed to send CAN message. Error code: ");
    Serial.println(result);
    return false; // Failed to send message
  }
}

//---------------------------------------------------------------------------------------------------------------------
// DESCRIPTION: Receives a CAN message if available.
//              If a message is available, it reads the message ID, data, and length into the provided variables.
// ARGUMENTS:
//   id: A reference to a variable to store the received message ID.
//   buf: A pointer to an array to store the received data bytes.
//   len: A reference to a variable to store the length of the received data.
// RETURN VALUE:
//   true if a message was received successfully, false otherwise.
bool receiveCANMessage(unsigned long &id, unsigned char *buf, unsigned char &len) {
  unsigned long startTime = millis(); // Start time for timeout
  
  // Receive message
  while (millis() - startTime < TIMEOUT_MS) { // Check for timeout
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      CAN.readMsgBuf(&id, &len, buf);
      Serial.print("Received Data: ");
      for (int i = 0; i < len; i++) {
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      return true; // Message received successfully
    }
  }
  
  Serial.println("Timeout: No CAN message received");
  return false; // Timeout occurred, no message received
}