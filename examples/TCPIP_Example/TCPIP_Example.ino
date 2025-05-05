/*
  Example of using the AMPX Modbus TCP/IP library
  to read data from an energy meter over Ethernet
  
  For ESP32 with Ethernet shield:
  - Connect the Ethernet shield's SPI pins to ESP32:
  - CS pin: GPIO5
*/

#include <SPI.h>
#include <Ethernet.h>
#include <ampx_modbus_tcpip.h>

// Debug mode - set to 1 to enable debug output, 0 to disable
#define DEBUG 1
#if DEBUG == 1
  #define debug(...) Serial.print(__VA_ARGS__)
  #define debugln(...) Serial.println(__VA_ARGS__)
#else
  #define debug(...)
  #define debugln(...)
#endif

// Define the Ethernet CS pin
#define ETH_SPI_CS 5

// Network settings
byte mac[] = {0x90, 0xA2, 0xDA, 0x0E, 0x94, 0xB5};  // MAC address for Ethernet shield
IPAddress ip(192, 168, 1, 177);                      // IP address for Arduino/ESP32
IPAddress gateway(192, 168, 1, 1);                   // Network gateway
IPAddress subnet(255, 255, 255, 0);                  // Subnet mask
IPAddress meter_ip(192, 168, 1, 178);                // Energy meter IP address

void setup() {
  #if DEBUG == 1
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  #endif
  
  debugln("AMPX Modbus TCP/IP Example");
  
  // Initialize Ethernet with CS pin
  Ethernet.init(ETH_SPI_CS);
  
  // Start Ethernet connection with static IP
  debugln("Starting Ethernet connection...");
  Ethernet.begin(mac, ip, gateway, subnet);
  
  // Give the Ethernet shield time to initialize
  delay(1000);
  
  // Check if hardware is connected
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    debugln("Ethernet shield not found! Check connections.");
    while (true) {
      delay(1);  // Do nothing, no point running without Ethernet hardware
    }
  }
  
  // Check if cable is connected
  if (Ethernet.linkStatus() == LinkOFF) {
    debugln("Ethernet cable is not connected! Check cable.");
  } else {
    debugln("Ethernet cable connected.");
  }
  
  // Print assigned IP address
  debug("My IP address: ");
  debugln(Ethernet.localIP());
  
  // Initialize Modbus with meter IP address
  modbus_init(meter_ip);
}

void loop() {
  uint16_t registerData[4];  // Buffer to hold register values
  
  // Test the connection to the Modbus server
  if (modbus_test_connection()) {
    debugln("Connection to Modbus server successful.");
  } else {
    debugln("Connection to Modbus server failed!");
    delay(5000);
    return;
  }
  
  debugln("\n--- Reading Serial Number (Register 70) ---");
  // Read meter serial number (registers 70-71)
  if (modbus_read_registers_tcpip(70, 2, registerData)) {
    // Print raw values
    debug("Register 70: 0x");
    debug(registerData[0], HEX);
    debug(", Register 71: 0x");
    debugln(registerData[1], HEX);
    
    // Combine registers in both endian formats
    unsigned long serial_big = combine_registers_integer(registerData[0], registerData[1], BIG_ENDIAN_FORMAT);
    unsigned long serial_little = combine_registers_integer(registerData[0], registerData[1], LITTLE_ENDIAN_FORMAT);
    
    debug("Serial Number (big-endian): ");
    debugln(serial_big);
    debug("Serial Number (little-endian): ");
    debugln(serial_little);
  } else {
    debugln("Failed to read serial number");
  }
  
  debugln("\n--- Reading Voltage (Register 1010) ---");
  // Read voltage on L1 (registers 1010-1011)
  if (modbus_read_registers_tcpip(1010, 2, registerData)) {
    // Print raw values
    debug("Register 1010: 0x");
    debug(registerData[0], HEX);
    debug(", Register 1011: 0x");
    debugln(registerData[1], HEX);
    
    // Interpret as float in both endian formats
    float voltage_big = combine_registers_float(registerData[0], registerData[1], BIG_ENDIAN_FORMAT);
    float voltage_little = combine_registers_float(registerData[0], registerData[1], LITTLE_ENDIAN_FORMAT);
    
    debug("Voltage L1 (big-endian): ");
    debug(voltage_big);
    debugln(" V");
    debug("Voltage L1 (little-endian): ");
    debug(voltage_little);
    debugln(" V");
  } else {
    debugln("Failed to read voltage");
  }
  
  debugln("\n--- Reading Current (Register 1000) ---");
  // Read current on L1 (registers 1000-1001)
  if (modbus_read_registers_tcpip(1000, 2, registerData)) {
    // Interpret as float in both endian formats
    float current_big = combine_registers_float(registerData[0], registerData[1], BIG_ENDIAN_FORMAT);
    float current_little = combine_registers_float(registerData[0], registerData[1], LITTLE_ENDIAN_FORMAT);
    
    debug("Current L1 (big-endian): ");
    debug(current_big);
    debugln(" A");
    debug("Current L1 (little-endian): ");
    debug(current_little);
    debugln(" A");
  } else {
    debugln("Failed to read current");
  }
  
  delay(5000);  // Wait 5 seconds before next read
} 