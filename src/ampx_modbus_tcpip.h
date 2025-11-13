#ifndef AMPX_MODBUS_TCPIP_H
#define AMPX_MODBUS_TCPIP_H

#include <Ethernet.h>
#include <SPI.h>

// Modbus TCP settings
const int MODBUS_PORT = 502;
const byte UNIT_ID = 1;  // Default Modbus unit ID
const int READ_HOLDING_REGISTERS = 3;  // Modbus function code 3
const int READ_INPUT_REGISTERS = 4;    // Modbus function code 4

// Endian format constants
const bool BIG_ENDIAN_FORMAT = true;
const bool LITTLE_ENDIAN_FORMAT = false;

// Global client for Modbus communication
extern EthernetClient modbusClient;
extern IPAddress modbusServerIP;  // Will be set by init function

// Function to combine two 16-bit registers into a 32-bit unsigned integer
// endianFormat: true for big-endian, false for little-endian
unsigned long combine_registers_integer(uint16_t reg1, uint16_t reg2, bool endianFormat);

// Function to combine two 16-bit registers into a 32-bit float
// endianFormat: true for big-endian, false for little-endian
float combine_registers_float(uint16_t reg1, uint16_t reg2, bool endianFormat, float scalingFactor = 1.0);

// Function to print byte as hex with leading zero
void printHex(byte b);

// Function to dump raw bytes for debugging
void dumpBytes(byte* buffer, int length);

// Initialize Modbus TCP client with server IP
void modbus_init(IPAddress serverIP);

// Function to test modbus connectivity, modbusClient.connected()
bool modbus_test_connection();

// Function to send Modbus TCP request
bool modbus_send_request(uint16_t startReg, uint16_t numRegs, uint8_t functionCode);

// Function to read Modbus TCP response
bool modbus_read_response(uint16_t* data, int expectedBytes);

// Function to read a register and return the value
bool modbus_read_registers_tcpip(uint16_t startReg, uint16_t numRegs, uint16_t* data, uint8_t functionCode = READ_HOLDING_REGISTERS);

#endif // AMPX_MODBUS_TCPIP_H 