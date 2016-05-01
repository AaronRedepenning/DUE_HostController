/* Program Name: DUE_HostController_v1
 * Author(s): Aaron Redepenning (rede3704@stthomas.edu)
 *            
 * Description: Host Controller for communicating with sensor modules
 *              over a CAN bus. Wireless transmits sensor data back to
 *              a raspberry pi using xbee modules.
 * 
 * Release Notes:
 *    [4/9/2016] - Version 1.0: Initial Creation
 */

#include <LiquidCrystal.h>
#include <Scheduler.h>
#include <due_can.h>
#include <Printers.h>
#include <XBee.h>
#include "PinMapping.h"
#include "SensorModules.h"

//////////////////////////////////////////////////////////////////////////
// Defines
//////////////////////////////////////////////////////////////////////////

#define VersionString "DUE_HostController Version 1.0 -> Initialized"
#define DEBUG 1           // Set to 1 for production build
#define MODULE_MAXIMUM 5 // Currently support 5 positions on the CAN bus
#define XBEE_MAX_PAYLOAD_SIZE 20
#define XBEE_QUEUE_SIZE 5
#define COORDINATOR_ADDR_16 0x0000

// T6 CAN Defines
#define T6_BOADCAST_ADDRESS   0xFF
#define T6_MY_ADDRESS         0x00
#define T6CMD_DISCOVERY       0x44
#define T6CMD_ASSIGN_ADDRESS  0x41
#define T6CMD_SAMPLE_REQUEST  0x53
#define T6CMD_SENSOR_UPDATE   0x55
#define T6CMD_ERROR           0x45

//////////////////////////////////////////////////////////////////////////
// Global Variables
//////////////////////////////////////////////////////////////////////////

SensorModule *SensorModules[MODULE_MAXIMUM] = { NULL }; 
uint8_t sensorModulesCount = 0;

// LCD Data
LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// CAN Data
CAN_FRAME RecieveFrame, ProvideAddressFrame, ReadSensorsFrame;
uint32_t canAddress; // Get rid of this 
static uint8_t _newCANAddress = 0x00;
uint8_t getAvailableAddress() {
  return ++_newCANAddress;
}

// XBee data
XBee xbee = XBee();
SensorModule *xbeeQueue[XBEE_QUEUE_SIZE] = { NULL };
uint8_t xbeeQueueHead = 0, xbeeQueueTail = 0, xbeeQueueCount = 0;

//////////////////////////////////////////////////////////////////////////
// Main Code
//////////////////////////////////////////////////////////////////////////

/**
 * Function: setup()
 */
void setup() {
  
#if DEBUG
  // Initialize Serial Port for Debugging
  Serial.begin(115200);
#endif

  // Initialize XBee Module and Serial for XBee communications
  Serial1.begin(9600);
  xbee.setSerial(Serial1);

  // Initialize LCD Display
  lcd.begin(16, 2);
  lcd.print("Host Controller");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize Can0, and set baudrate
  Can0.begin(CAN_BPS_500K);
  
  // Setup CAN Filter to allow any extended messages through
  // TODO: Change this in the future to restricted addresses
  for(int filter = 0; filter < 3; filter++) {
    Can0.setRXFilter(filter, 0, 0, true);
  }

  // Setup CAN Message frames - Assign Address Frame
  uint32_t id = T6_BOADCAST_ADDRESS;
  id = (id << 21) | T6CMD_ASSIGN_ADDRESS;
  ProvideAddressFrame.id = id;
  ProvideAddressFrame.extended = true;
  ProvideAddressFrame.length = 7;

  // Setup CAN Message frames - Read Sensors Frame
  id = 0xBB;
  id = (id << 21) | T6CMD_SAMPLE_REQUEST;
  ReadSensorsFrame.id = id;
  ReadSensorsFrame.extended = true;
  ReadSensorsFrame.length = 0;
  
#if DEBUG
  Serial.println(VersionString);
#endif
  delay(3000); // Wait for things to settle
  
  // Start all threads, loop() is started automatically
  Scheduler.startLoop(loop2); // Timer thread
  Scheduler.startLoop(loop3); // UI thread
  Scheduler.startLoop(loop4); // XBee thread
}

/**
 * Function: loop()
 * Description: CAN Messaging Thread
 */
void loop() {
  
  if(Can0.available() > 0) {
    Can0.read(RecieveFrame);

    // Parse Can ID
    uint8_t command = RecieveFrame.id & 0x000000FF;
    uint8_t sender = (RecieveFrame.id >> 8) & 0x000000FF;
    uint8_t destination = (RecieveFrame.id >> 21) & 0x000000FF;

    switch(command) {
      case T6CMD_DISCOVERY: // Discovery Command
      {
#if DEBUG
        Serial.println("Discovery Command");
#endif

        // Build Command
        for(int i = 0; i < 6; i++) {
          ProvideAddressFrame.data.bytes[i] = RecieveFrame.data.bytes[i + 2]; // Load device MAC Address
        }
        canAddress = getAvailableAddress(); // Get a new Address from the provider
        ProvideAddressFrame.data.bytes[6] = canAddress; // CAN Address Provided
        Can0.sendFrame(ProvideAddressFrame); // Give sensor module its address

        // Place new sensor module in array, at its specified height (DIP Switch setting)
        SensorModule *module = new SensorModule("Module " + String(RecieveFrame.data.bytes[0]), canAddress, RecieveFrame.data.bytes[0]);
        SensorModules[sensorModulesCount] = module;
        sensorModulesCount++;
        break;
      }
      case T6CMD_SENSOR_UPDATE: // Sensor Update Command
      {
#if DEBUG
        Serial.println("Sensor Update Command");
#endif
        updateSensor(RecieveFrame);        
        break;
      }
      default: { break; } // Host Controller doesn't handle any other commands
    }
  }

  yield(); // Call this allow other threads to run
}

/**
 * Function: loop2()
 * Description: Timer thread
 */
void loop2() {
  if(sensorModulesCount > 0) {
#if DEBUG
    Serial.println("Requested sensor reading");
#endif
    uint32_t id = canAddress;
    id = (id << 21) | 0x53;
    ReadSensorsFrame.id = id;
    Can0.sendFrame(ReadSensorsFrame);
    delay(10000); // 10 second sampling
  }
  yield();
}

/**
 * Function: loop3()
 * Description: UI Thread
 */
void loop3() {
  for(int i = 0; i < sensorModulesCount; i++) {
    lcd.clear();
    lcd.print(SensorModules[i]->name);
    lcd.setCursor(0, 1);
    lcd.print("Temp: "); lcd.print(SensorModules[i]->temperature); lcd.print(" F");
    delay(10000); // Wait 10s

    lcd.clear();
    lcd.print(SensorModules[i]->name);
    lcd.setCursor(0, 1);
    lcd.print("Humi: "); lcd.print(SensorModules[i]->humidity); lcd.print(" %RH");
    delay(10000); // Wait 10s

    lcd.clear();
    lcd.print(SensorModules[i]->name);
    lcd.setCursor(0, 1);
    lcd.print("Pres: "); lcd.print(SensorModules[i]->pressure); lcd.setCursor(12, 1); lcd.print(" hPa");
    delay(10000); // Wait 10s
  }
  yield();
}

void updateSensor(CAN_FRAME &frame) {
  // Parse Can ID
  uint8_t command = frame.id & 0x000000FF;
  uint8_t sender = (frame.id >> 8) & 0x000000FF;
  uint8_t destination = (frame.id >> 21) & 0x000000FF;

  float temperature, humidity, pressure;
  uint16_t sensorData = frame.data.bytes[0];
  sensorData = (sensorData << 8) | frame.data.bytes[1];
  temperature = sensorData / 10.0;

  humidity = frame.data.bytes[2];

  sensorData = frame.data.bytes[3];
  sensorData = (sensorData << 8) | frame.data.bytes[4];
  pressure = sensorData / 10.0;

  // Find the sensor module to update the sensor readings
  for(int i = 0; i < MODULE_MAXIMUM; i++) {
    if((SensorModules[i] != NULL) && (SensorModules[i]->address == sender)) {
      // Found sensor, now update the readings
#if DEBUG
      Serial.println("Updated Reading in Array");
#endif
      SensorModules[i]->UpdateReadings(temperature, humidity, pressure, NULL);

      // Enqueue Sensor module to be send over xbee
      xbeeQueue[xbeeQueueTail] = SensorModules[i];
      xbeeQueueTail++;
      if(xbeeQueueTail >= XBEE_QUEUE_SIZE) {
        xbeeQueueTail = 0;
      }
      xbeeQueueCount++;
    }
  }
  
#if DEBUG
  Serial.print("Command: "); Serial.println(command, HEX);
  Serial.print("Sender : "); Serial.println(sender, HEX);
  Serial.print("Destin : "); Serial.println(destination, HEX);
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" deg F");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %RH");
  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" kPa");
  Serial.println();
#endif
}

/**
 * Function: loop4()
 * Description: XBee Thread
 */
uint8_t payload[XBEE_MAX_PAYLOAD_SIZE];
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40eb3589); // TODO: Should be able to use broadcast address instead, 0x0000
uint16_t util;

void loop4() {
  // Check if any SensorModules have been added to the queue
  if(xbeeQueueCount > 0) {
    // Data in the queue, packet it and send over XBee network
    SensorModule *sm = xbeeQueue[xbeeQueueHead];

    // Build xbee data packet
    util = (uint16_t)(sm->temperature * 10.0); // Temperature
    payload[0] = (util >> 8) & 0x00FF;
    payload[1] = util & 0x00FF;
    payload[2] = sm->humidity;  // Humidity
    util = sm->pressure * 10;   // Pressure
    payload[3] = (util >> 8) & 0x00FF;
    payload[4] = util & 0x00FF;

    // Send data with xbee
#if DEBUG
    Serial.print("Sending XBee Frame...");
#endif
    //Tx16Request tx = Tx16Request(COORDINATOR_ADDR_16, payload, 5);
    ZBTxRequest zbTx = ZBTxRequest(addr64, payload, 5);
    xbee.send(zbTx);

    // Remove sensor module from queue
    xbeeQueue[xbeeQueueHead] = NULL;
    xbeeQueueHead++;
    if(xbeeQueueHead >= XBEE_QUEUE_SIZE) {
      xbeeQueueHead = 0;
    }
    xbeeQueueCount--;
  }
  yield();
}






