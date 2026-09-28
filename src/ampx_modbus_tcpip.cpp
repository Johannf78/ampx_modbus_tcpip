#include "ampx_modbus_tcpip.h"

/*
  AMPX Modbus TCP/IP Library
  
  This library provides functions for communicating with Modbus TCP/IP devices,
  particularly energy meters, over Ethernet. It handles:
  
  - Initialization of Ethernet and Modbus TCP connections
  - Reading holding registers from Modbus devices
  - Converting register data to various formats (integers, floats)
  - Debug output for troubleshooting
  
  The library is designed to work with ESP32 and Arduino boards with Ethernet
  capabilities. It requires the Ethernet library to be installed.
  
  Created by AMPX Energy
  https://ampx.co
*/

//Enable or disable debug output, set to 0 to disable debug output (saves memory)
#define DEBUG 0  
#if DEBUG == 1
  #define debug(...) Serial.print(__VA_ARGS__)
  #define debugln(...) Serial.println(__VA_ARGS__)
#else
  #define debug(...)
  #define debugln(...)
#endif

// Global client for Modbus communication
EthernetClient modbusClient;
IPAddress modbusServerIP;  // Will be set by init function
void (*modbusIdleCallback)() = nullptr;

void modbus_set_idle_callback(void (*callback)()) {
  modbusIdleCallback = callback;
}

// Function to combine two 16-bit registers into a 32-bit unsigned integer
// endianFormat: true for big-endian, false for little-endian
unsigned long combine_registers_integer(uint16_t reg1, uint16_t reg2, bool endianFormat) {
  if (endianFormat == BIG_ENDIAN_FORMAT) {
    // Big-endian: first register is high word, second register is low word
    return ((unsigned long)reg1 << 16) | reg2;
  } else {
    // Little-endian: first register is low word, second register is high word
    return ((unsigned long)reg2 << 16) | reg1;
  }
}

// Function to combine two 16-bit registers into a 32-bit float
// endianFormat: true for big-endian, false for little-endian
float combine_registers_float(uint16_t reg1, uint16_t reg2, bool endianFormat, float scalingFactor) {
  unsigned long combined = combine_registers_integer(reg1, reg2, endianFormat);
  return combined * scalingFactor;
}

// Function to print byte as hex with leading zero
void printHex(byte b) {
  if (b < 0x10) debug("0");
  debug(b, HEX);
}

// Function to dump raw bytes for debugging
void dumpBytes(byte* buffer, int length) {
  debug("Raw bytes: ");
  for (int i = 0; i < length; i++) {
    printHex(buffer[i]);
    debug(" ");
  }
  debugln("");
}

// Initialize Modbus TCP client with server IP
void modbus_init(IPAddress serverIP) {
  modbusServerIP = serverIP;
  debug("Modbus server IP set to: ");
  debugln(modbusServerIP);
}

// Function to test modbus connectivity, modbusClient.connected()
bool modbus_test_connection() {


  if (modbusClient.connected()) {
    debugln("Already connected");
    return true;
  }
  
  debug("Testing connection to Modbus server... ");
  if (modbusClient.connect(modbusServerIP, MODBUS_PORT)) {
    debugln("Success!");
    //Don't stop the connection, it will be closed by the caller
    //modbusClient.stop();
    return true;
  } else {
    debugln("Failed!");
    return false;
  }
}

// Function to send Modbus TCP request
bool modbus_send_request(uint16_t startReg, uint16_t numRegs, uint8_t functionCode) {
  byte requestBuffer[12];  // MBAP (6 bytes) + PDU (6 bytes)
  static uint16_t transactionId = 1;  // Start from 1 instead of 0
  
  // Build request in buffer
  requestBuffer[0] = highByte(transactionId);
  requestBuffer[1] = lowByte(transactionId);
  requestBuffer[2] = 0x00;  // Protocol ID high byte
  requestBuffer[3] = 0x00;  // Protocol ID low byte
  requestBuffer[4] = 0x00;  // Length high byte
  requestBuffer[5] = 0x06;  // Length low byte (6 bytes following)
  requestBuffer[6] = UNIT_ID;
  requestBuffer[7] = functionCode;
  requestBuffer[8] = highByte(startReg);
  requestBuffer[9] = lowByte(startReg);
  requestBuffer[10] = highByte(numRegs);
  requestBuffer[11] = lowByte(numRegs);
  
  transactionId++;  // Increment for next request
  
  // Reuse the socket opened by modbus_test_connection(). Connect only if it dropped.
  // If the socket is still connected, don't connect again.
  if (!modbusClient.connected()) {
  
    debug("Connecting to ");
    debug(modbusServerIP);
    debug(":");
    debugln(MODBUS_PORT);

    if (!modbusClient.connect(modbusServerIP, MODBUS_PORT)) {
      debugln("Connection failed");
      return false;
    }
  }
  
  
  // Send request
  debugln("Sending request:");
  dumpBytes(requestBuffer, 12);
  modbusClient.write(requestBuffer, 12);
  
  return true;
}

// Function to read Modbus TCP response
bool modbus_read_response(uint16_t* data, int expectedBytes) {
  unsigned long timeout = millis() + 2000; // 2 second timeout
  int bytesRead = 0;
  byte buffer[256];  // Response buffer
  
  debugln("Waiting for response...");

  // Keep reading until the MBAP length says the frame is complete.
  // Breaking on the first bytes (after delay(100) was removed) treated a partial
  // packet as a failed reply and closed the socket. The browser WebSocket then
  // missed its handshake and the page stopped on Disconnected.
  int frameBytes = 0;
  while (millis() < timeout) {
    while (modbusClient.available() && bytesRead < (int)sizeof(buffer)) {
      buffer[bytesRead] = modbusClient.read();
      bytesRead++;
    }

    if (frameBytes == 0 && bytesRead >= 6) {
      uint16_t mbapLength = ((uint16_t)buffer[4] << 8) | buffer[5];
      // Length counts the bytes after the 6-byte MBAP header. A usable reply
      // is at least unit id + function + one more byte (9 bytes total).
      if (mbapLength < 3 || mbapLength > 250) {
        debugln("Error: Bad MBAP length");
        modbusClient.stop();
        return false;
      }
      frameBytes = 6 + mbapLength;
    }

    if (frameBytes > 0 && bytesRead >= frameBytes) {
      break;
    }

    if (modbusIdleCallback) {
      modbusIdleCallback();
    }
    delay(1);
  }
  
  //modbusClient.stop();  // Close connection after reading
  
// A normal reply stays on this socket so the next register does not open port 502 again.
// A timeout or a short frame leaves unread bytes, so close and let the next register reconnect.
// 9 bytes is the shortest frame this parser can inspect:
// 7-byte MBAP header (transaction id, protocol id, length, unit id)
// plus function code and byte count (or exception code) at buffer[6..8].
// A real register value is longer (9 + 2 bytes per register); that is checked later.
// Fewer than 9 bytes is a timeout or a fragment. Close so those bytes cannot
// sit on the socket and corrupt the next register. A full reply stays open.
if (bytesRead < 9) {
  debugln("Error: Response timeout or short frame");
  modbusClient.stop();
  return false;
}
  
  debug("Received ");
  debug(bytesRead);
  debugln(" bytes:");
  dumpBytes(buffer, bytesRead);
  
  // Check minimum length
  /*
  if (bytesRead < 9) {
    debugln("Error: Response too short");
    return false;
  }
  */
  
  // Parse response
  uint16_t transId = (buffer[0] << 8) | buffer[1];
  uint16_t protId = (buffer[2] << 8) | buffer[3];
  uint16_t length = (buffer[4] << 8) | buffer[5];
  uint8_t unitId = buffer[6];
  uint8_t funcCode = buffer[7];
  
  // Debug output
  debugln("\nResponse details:");
  debug("Transaction ID: 0x"); debugln(transId, HEX);
  debug("Protocol ID: 0x"); debugln(protId, HEX);
  debug("Length: "); debugln(length);
  debug("Unit ID: "); debugln(unitId);
  debug("Function Code: 0x"); debugln(funcCode, HEX);
  
  // Check for Modbus exception response
  if (funcCode & 0x80) {
    debug("Error: Modbus exception received. Code: 0x");
    debugln(buffer[8], HEX);
    return false;
  }
  
  // Get byte count
  uint8_t byteCount = buffer[8];
  debug("Byte Count: "); debugln(byteCount);
  
  // Extract register values if we have enough data
  if (byteCount > 0 && bytesRead >= 9 + byteCount) {
    for (int i = 0; i < byteCount/2 && i < 16; i++) {  // Limit to 16 registers max
      data[i] = (buffer[9 + i*2] << 8) | buffer[10 + i*2];
      debug("Register value ");
      debug(i);
      debug(": 0x");
      debugln(data[i], HEX);
    }
    return true;
  } else {
    debugln("Error: Not enough data in response");
    // Header arrived without the register bytes. Close so the remainder
    // cannot be read as the next register's reply. If the socket is not connected, don't close it.
    modbusClient.stop();
    return false;
  }
}

// Function to read a register and return the value.
// Does not call modbus_test_connection(): that is connect+stop.
// Callers that want a health check call it themselves (e.g. serviceMeters).
// Connect failure is handled in modbus_send_request().
bool modbus_read_registers_tcpip(uint16_t startReg, uint16_t numRegs, uint16_t* data, uint8_t functionCode) {
  if (modbus_send_request(startReg, numRegs, functionCode)) {
    
    // No fixed pause here. delay(100) ran before every register (~20 × 100 ms, about 2 s)
    // even when the meter had already answered. modbus_read_response() waits up to 2 seconds
    // and returns as soon as the frame arrives, so the pause was not needed to catch the reply.
    //delay(100);  // Wait for response

    return modbus_read_response(data, numRegs * 2);
  }

  return false;
}