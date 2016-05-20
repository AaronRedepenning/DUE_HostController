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

#include <Wire.h>
#include <LiquidCrystal.h>
#include <Scheduler.h>
#include <due_can.h>
#include <Printers.h>
#include <XBee.h>
#include <Opt3001.h>
#include "PinMapping.h"
#include "SensorModules.h"

//////////////////////////////////////////////////////////////////////////
// Defines
//////////////////////////////////////////////////////////////////////////

#define VersionString         "DUE_HostController Version 1.0 -> Initialized"
#define DEBUG 1 // Set to 1 for production build
#define MODULE_MAXIMUM        5 // Currently support 5 positions on the CAN bus
#define XBEE_MAX_PAYLOAD_SIZE 60
#define XBEE_QUEUE_SIZE       5
#define COORDINATOR_ADDR_16   0x0000

// T6 Protocol CAN Defines
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

struct Point {
  uint8_t x;
  uint8_t y;
};

/************************* All configurable settings here ********************************/
unsigned long samplingRate = 10000; // Change to configure sampling rate of sensor modules
const String nodeName = "Team 6 Sensor Node";
const Point nodePosition = {.x = 2, .y = 5};

// TODO: These settings will eventually be read off a file on an SD card to allow easier
//        configuration without having to reprogram the device every time you want to 
//        change them.
/****************************************************************************************/

SensorModule *SensorModules[MODULE_MAXIMUM] = { NULL }; 
uint8_t sensorModulesCount = 0;

// LCD Data
LiquidCrystal lcd(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// CAN Data
CAN_FRAME RecieveFrame, ProvideAddressFrame, ReadSensorsFrame; // CAN Frame types
 
static uint8_t _newCANAddress = 0x00;
uint8_t getAvailableAddress() {
  return ++_newCANAddress; // First provided address is 0x01, it will increment each time
}

// XBee data
XBee xbee = XBee();
SensorModule *xbeeQueue[XBEE_QUEUE_SIZE] = { NULL }; // Sensor modules queue for sending over xbee
uint8_t xbeeQueueHead = 0, xbeeQueueTail = 0, xbeeQueueCount = 0;
uint8_t sampleId;

// OPT3001 Light Sensor Data
Opt3001 opt3001;
uint32_t lightIntensity = 0;
bool lightHasNewReading = false;

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
  Serial.println(VersionString);
#endif

  // Initialize XBee Module and Serial for XBee communications
  Serial1.begin(9600);
  xbee.setSerial(Serial1);
#if DEBUG
  Serial.println("XBee Initialized----------------------------------");
#endif

  // Initialize LCD Display
  lcd.begin(16, 2);
  lcd.print("Host Controller");
  lcd.setCursor(0, 1);
  lcd.print("Initializing");

#if DEBUG
  Serial.println("LCD Initialized----------------------------------");
#endif

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
  id = T6_BOADCAST_ADDRESS;
  id = (id << 21) | T6CMD_SAMPLE_REQUEST;
  ReadSensorsFrame.id = id;
  ReadSensorsFrame.extended = true;
  ReadSensorsFrame.length = 0;
#if DEBUG
  Serial.println("CAN Initialized----------------------------------");
#endif

  // Setup OPT3001 and I2C Bus
  Wire.begin();
  opt3001.begin();
#if DEBUG
  Serial.println("OPT3001 Initialized----------------------------------");
  
  // get manufacturer ID from OPT3001. Default = 0101010001001001
  lightIntensity = opt3001.readManufacturerId();  
  Serial.print("Manufacturer ID: "); 
  Serial.println(lightIntensity, BIN);

  // get device ID from OPT3001. Default = 0011000000000001
  lightIntensity = opt3001.readDeviceId();  
  Serial.print("Device ID: "); 
  Serial.println(lightIntensity, BIN);
  
  // read config register from OPT3001. Default = 1100110000010000
  lightIntensity = opt3001.readConfigReg();  
  Serial.print("Configuration Register: "); 
  Serial.println(lightIntensity, BIN);

  // read Low Limit register from OPT3001. Default = 0000000000000000
  lightIntensity = opt3001.readLowLimitReg();  
  Serial.print("Low Limit Register: "); 
  Serial.println(lightIntensity, BIN);
  
  // read High Limit register from OPT3001. Default = 1011111111111111
  lightIntensity = opt3001.readHighLimitReg();  
  Serial.print("High Limit Register: "); 
  Serial.println(lightIntensity, BIN);    

  lightIntensity = 0;
  
  Serial.println("\nOPT3001 READINGS-------------------------------------");
  Serial.println();
#endif

  // Start all threads, loop() is started automatically
  // loop -> CAN Messaging thread
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

        // Check if sensor module already exists in the array
        uint8_t canAddress = 0; 
        int16_t indexInArray = -1;
        for(int i = 0; i < sensorModulesCount; i++) {
          if(SensorModules[i]->EqualMacAddresses(&RecieveFrame.data.bytes[2])) {
            canAddress = SensorModules[i]->canAddress;
            indexInArray = i;
            break;
          }
        }
        
        if(canAddress == 0) {
          canAddress = getAvailableAddress(); // New sensor module so get a new can address from the provider
        }

        // Build Command
        for(int i = 0; i < 6; i++) {
          ProvideAddressFrame.data.bytes[i] = RecieveFrame.data.bytes[i + 2]; // Load device MAC Address
        }
        ProvideAddressFrame.data.bytes[6] = canAddress; // CAN Address Provided
        Can0.sendFrame(ProvideAddressFrame); // Give sensor module its address

        // Place new sensor module in array, at its specified height (DIP Switch setting)
        SensorModule *module = new SensorModule("Module " + String(RecieveFrame.data.bytes[0]), &RecieveFrame.data.bytes[2], 
                                      canAddress, RecieveFrame.data.bytes[0], RecieveFrame.data.bytes[1]);
        if(indexInArray >= 0) { // Add at same spot in array is it was already there
          SensorModules[indexInArray] = module;
#if DEBUG
          Serial.println("Sensor Module re-added");
#endif
        } 
        else { // Add at a new spot if this is a new sensor module
          SensorModules[sensorModulesCount] = module;
          sensorModulesCount++;
#if DEBUG
          Serial.println("Sensor module added");
#endif
        }
        break;
      }
      case T6CMD_SENSOR_UPDATE: // Sensor Update Command
      {
#if DEBUG
        Serial.println("Sensor Update Command");
#endif
        updateSensorModule(RecieveFrame);        
        break;
      }
      default: { break; } // Host Controller doesn't handle any other commands
    }
  }

  yield(); // Call this allow other threads to run
}

void updateSensorModule(CAN_FRAME &frame) {
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
    if((SensorModules[i] != NULL) && (SensorModules[i]->canAddress == sender)) {
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
      break;
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
 * Function: loop2()
 * Description: Timer thread
 */
void loop2() {
  // Increment sample id with each sample interval
  // TODO: this should be sample time with RTC
  sampleId++;
  
  // Read light sensor
  lightIntensity = opt3001.readResult();
  lightHasNewReading = true;
#if DEBUG
  Serial.print("LUX Readings = ");
  Serial.println(lightIntensity, DEC);
#endif

  // See if there are sensor modules to get readings from
  if(sensorModulesCount > 0) {
#if DEBUG
    Serial.println("Requested sensor reading");
#endif
    Can0.sendFrame(ReadSensorsFrame);
  }
  
  delay(samplingRate); // Wait sampling rate time between readings
}

/**
 * Function: loop3()
 * Description: UI Thread
 * TODO: Create a better display loop that appears to update more quickly
 */
const unsigned long displayTime = 10000; // 10 second diplay time = 10000 milliseconds
const uint8_t lcdRefreshRate = 250; // Refresh rate in milliseconds
void loop3() {
  static unsigned long startTime;
  
  // Display Light Intensity from OPT3001
  startTime = millis(); // Get current time
  while(abs(millis() - startTime) < displayTime) {
    lcd.clear();
    lcd.print("Host Controller");
    lcd.setCursor(0, 1);
    lcd.print("Light: "); lcd.print(lightIntensity); lcd.print(" lux");
    delay(lcdRefreshRate);
  }

  // Display sensor readings from sensor modules (Temp, Hum, Press, Co2)
  for(int i = 0; i < sensorModulesCount; i++) {
    
    startTime = millis();
    while(abs(millis() - startTime) < displayTime) {
      lcd.clear();
      lcd.print(SensorModules[i]->name);
      lcd.setCursor(0, 1);
      lcd.print("Temp: "); lcd.print(SensorModules[i]->temperature); lcd.print(" F"); // Degrees F
      delay(lcdRefreshRate);
    }
    
    startTime = millis();
    while(abs(millis() - startTime) < displayTime) {
      lcd.clear();
      lcd.print(SensorModules[i]->name);
      lcd.setCursor(0, 1);
      lcd.print("Humi: "); lcd.print(SensorModules[i]->humidity); lcd.print(" %RH"); // Relative humidity
      delay(lcdRefreshRate);
    }
    
    startTime = millis();
    while(abs(millis() - startTime) < displayTime) {
      lcd.clear();
      lcd.print(SensorModules[i]->name);
      lcd.setCursor(0, 1);
      lcd.print("Pres: "); lcd.print(SensorModules[i]->pressure); lcd.setCursor(12, 1); lcd.print(" hPa"); // Hectopascalls
      delay(lcdRefreshRate);
    }
    
    startTime = millis();
    while(SensorModules[i]->hasCo2 && (abs(millis() - startTime) < displayTime)) {
      lcd.clear();
      lcd.print(SensorModules[i]->name);
      lcd.setCursor(0, 1);
      lcd.print("Co2: "); lcd.print(SensorModules[i]->carbonDioxide); lcd.print(" ppm"); // Parts-per-million
      delay(lcdRefreshRate);
    }
  }
  
  yield();
}

/**
 * Function: loop4()
 * Description: XBee Thread
 */
uint8_t payload[XBEE_MAX_PAYLOAD_SIZE];
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40fb6a8d); // TODO: Should be able to use broadcast address instead, 0x0000

void loop4() {
  uint16_t util, index;
  
  // Check if any SensorModules have been added to the queue
  if(xbeeQueueCount > 0) {
    // Get a pointer to the next sensor module in the queue
    SensorModule *sm = xbeeQueue[xbeeQueueHead];

    index = 0;
    payload[index++] = sampleId;
    for(index = 1; index < nodeName.length() + 1; index++) {
      payload[index] = nodeName[index - 1]; // Load node name
    }
    payload[index++] = '\0'; // end of node name
    payload[index++] = nodePosition.x; // x-position
    payload[index++] = nodePosition.y; // y-position
    payload[index++] = 'U'; // Update command
    payload[index++] = sm->hasCo2 ? 0x02 : 0x01; // Light reading
    for(int i = 0; i < sm->name.length(); i++, index++) {
      payload[index] = sm->name[i];
    }
    payload[index++] = '\0';
    for(int i = 0; i < 6; i++, index++) {
      payload[index] = sm->macAddress[i];
    }
    payload[index++] = sm->height;
    
    util = sm->temperature * 10;
    payload[index++] = (util >> 8);
    payload[index++] = util;
    payload[index++] = sm->humidity;
    util = sm->pressure * 10;
    payload[index++] = (util >> 8);
    payload[index++] = util;
    
    if(sm->hasCo2) {
      util = sm->carbonDioxide * 10;
      payload[index++] = (util >> 8);
      payload[index++] = util;
    }

    // Send data with xbee
#if DEBUG
    Serial.println("Sending XBee Frame...SensorModule");
#endif
    //Tx16Request tx = Tx16Request(COORDINATOR_ADDR_16, payload, 5);
    ZBTxRequest zbTx = ZBTxRequest(addr64, payload, index);
    xbee.send(zbTx);

    // Remove sensor module from queue
    xbeeQueue[xbeeQueueHead] = NULL;
    xbeeQueueHead++;
    if(xbeeQueueHead >= XBEE_QUEUE_SIZE) {
      xbeeQueueHead = 0;
    }
    xbeeQueueCount--;
  }
  
  // Check if there is a new light reading to send
  if(lightHasNewReading) {
    // Create data packet for light intensity reading
    // Format: See Doc.
    index = 0;
    payload[index++] = sampleId;
    for(index = 1; index < nodeName.length() + 1; index++) {
      payload[index] = nodeName[index - 1]; // Load node name
    }
    payload[index++] = '\0'; // end of node name
    payload[index++] = nodePosition.x; // x-position
    payload[index++] = nodePosition.y; // y-position
    payload[index++] = 'U'; // Update command
    payload[index++] = 0x00; // Light reading
    payload[index++] = (lightIntensity >> 24); // Light intensity reading
    payload[index++] = (lightIntensity >> 16);
    payload[index++] = (lightIntensity >> 8);
    payload[index++] = lightIntensity;
    
#if DEBUG
    Serial.println("Sending XBee Frame...Light");
#endif
    //Tx16Request tx = Tx16Request(COORDINATOR_ADDR_16, payload, 5);
    ZBTxRequest zbTx = ZBTxRequest(addr64, payload, index);
    xbee.send(zbTx);
    
    lightHasNewReading = false;  
  }
  yield();
}






