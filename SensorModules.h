#include <Arduino.h>

class SensorModule {
    
public:
    // Sensor Module Data
    String          name;
    uint8_t         macAddress[6];
    uint8_t         canAddress;
    uint8_t         height;
    double          temperature;
    uint8_t         humidity;
    double          pressure;
    bool            hasCo2;    
    double          carbonDioxide;
    
    // Default Constructor
    SensorModule(String name, uint8_t *macAddress, uint8_t address, uint8_t height, bool hasCo2);   
    
    // Public Functions
    void UpdateReadings(double temp, uint8_t hum, double press, double co2); 
    bool EqualMacAddresses(uint8_t *otherMac);  
};
