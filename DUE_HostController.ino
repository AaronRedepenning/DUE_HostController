/* Program Name: DUE_HostController_v1
 * Author(s): Aaron Redepenning (rede3704@stthomas.edu)
 *            Gene Binfa        ()
 *            Sam Hanowski      ()
 * Description: Firmware for Arduino Due to perform the following:
 *                * Read Sensor Modules over CAN bus
 *                * Save sensor data to SD card in FAT32 format
 *                * Display current sensor data on 16x2 LCD
 *                * Send sensor data wirelessly over XBee network
 *                  to Rasberry Pi Gateway
 * Release Notes:
 *    [4/9/2016] - Version 1.0: Initial Creation
 */

#include <LiquidCrystal.h>
#include <Scheduler.h>
#include <due_can.h>
#include "SensorModules.h"
#include "PinMapping.h"

#define CAN_NO_ENABLE_PIN                 255
#define CAN_BAUDRATE_500KHZ               500000
#define CAN_SENSOR_UPDATE_MESSAGE_LENGTH  5
#define CAN_SERVER_ADDRESS                0x00
#define CAN_BROADCAST_ADDRESS             0xFF
#define CAN_CMD_NOT_AWK                   'N'
#define CAN_CMD_AWK                       'A'
#define CAN_CMD_DISCOVERY                 'D'
#define CAN_CMD_SAMPLE_SENSORS            'S'
#define CAN_CMD_SENSOR_UPDATE             'U'

/************************ Global Variables ******************************/
// LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
SensorModuleList SensorModules;
CAN_FRAME rxFrame;
CAN_FRAME txFrame;
CAN_FRAME requestFrame;

void setup() {
  Serial.begin(115200);
  
  // Initialize CAN Port 0, and watch for all CAN Bus Traffic
  Can0.begin(CAN_BAUDRATE_500KHZ, CAN_NO_ENABLE_PIN);
  Can0.watchFor();
  Can1.disable(); // Not using CAN port 1

  // Initialize Liquid Crystal Display
  // lcd.begin(16, 1);
  // lcd.print("Welcome");

  // Initialize Request Frame
  requestFrame.id = CAN_BROADCAST_ADDRESS;
  requestFrame.id = (requestFrame.id << 13) | CAN_SERVER_ADDRESS;
  requestFrame.id = (requestFrame.id << 8) | CAN_CMD_SAMPLE_SENSORS;
  requestFrame.extended = true;
  requestFrame.length = 0;

  Scheduler.startLoop(loop1);
  Scheduler.startLoop(loop2);
}

// Main Loop - Used for receiving CAN Messages
uint8_t senderAddress;
uint8_t receiverAddress;
uint8_t command;
void loop() {
  if(Can0.available() > 0) {
    // Message available, figure out what type of message it is
    Can0.read(rxFrame);
    Serial.println("CAN0 Available");
    senderAddress = (rxFrame.id >> 8) & 0x0000FF;
    receiverAddress = (rxFrame.id >> 21) & 0x0000FF;
    command = rxFrame.id & 0x0000FF;
    Serial.print("0x");
    Serial.println(command, HEX);
    switch(command) {
      case CAN_CMD_DISCOVERY:
        // Network discovery command
        CanNetworkDiscoveryCommand(senderAddress);
        break;
      case CAN_CMD_SENSOR_UPDATE:
        // Sensor update command
        SensorUpdateCommand(senderAddress, rxFrame.data.bytes, rxFrame.length);
        break;
      default:
        // Command not recognized
        break;
    }
  }
  yield();
}

// Ask for sensor updates task
void loop1() {
  if(SensorModules.Count() > 0) {
    Can0.sendFrame(requestFrame);
    delay(100000); // Delay 1 minute
  }
  else {
    yield();
  }
}

void loop2() {
  Serial.println("UI Loop Tick");
  delay(5000);
}

bool SensorUpdateCommand(uint8_t senderAddress, uint8_t *data, uint8_t len) {
  uint16_t tmp;
  Serial.println("Sensor Update Recieved");
  if(len != CAN_SENSOR_UPDATE_MESSAGE_LENGTH) {
    // Invalid Message since data buffer is not expected length
    return false;
  }

  // Find sensor module, it should already exist in the SensorModuleList
  SensorModule *module = SensorModules.Get(senderAddress);
  if(module == NULL) {
    // Error: Module not found
    return false;
  }

  tmp = data[0]; // Temperature high bits
  tmp = (tmp << 8) | data[1]; // Temperature low bits
  module->temperature = tmp / 10.0;
  module->humidity = data[2];
  tmp = data[3]; // Pressure high bits
  tmp = (tmp << 8) | data[4]; // Pressure low bits
  module->pressure = tmp / 10.0;

  return true;
}

bool CanNetworkDiscoveryCommand(uint8_t senderAddress) {
  // Check if senderAddress exists in SensorModuleList
  Serial.println("Network Discovery Received");
  if(SensorModules.Get(senderAddress) != NULL) {
    // This address is already being used
    txFrame.id = 0 | (senderAddress << 21);
    txFrame.id |= (CAN_SERVER_ADDRESS << 8);
    txFrame.id |= CAN_CMD_NOT_AWK;
    txFrame.length = 0;
    txFrame.extended = true;
  }
  else {
    // This address is free
    txFrame.id = 0 | (senderAddress << 21);
    txFrame.id |= (CAN_SERVER_ADDRESS << 8);
    txFrame.id |= CAN_CMD_AWK;
    txFrame.length = 0;
    txFrame.extended = true;
  }

  return Can0.sendFrame(txFrame);
}








