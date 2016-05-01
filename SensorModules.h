#include <Arduino.h>

class SensorModule {
    
public:
    // Sensor Module Data
    String          name;
    uint8_t         address;
    uint8_t         height;
    double          temperature;
    uint8_t         humidity;
    double          pressure;
    double          carbonDioxide;
    
    // Default Constructor
    SensorModule(String name, uint8_t address, uint8_t height);   
    
    // Public Functions
    void UpdateReadings(double temp, uint8_t hum, double press, double co2);    
};
