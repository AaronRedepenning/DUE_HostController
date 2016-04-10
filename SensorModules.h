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
    SensorModule    *next;
    
    // Default Constructor
    SensorModule(String name, uint8_t address, uint8_t height);   
    
    // Public Functions
    void UpdateReadings(double temp, uint8_t hum, double press, double co2);    
};

class SensorModuleList {
    
public:
    // Default Constructor
    SensorModuleList();
    
    // Public Functions
    SensorModule *Get(uint8_t id);
    SensorModule *GetByIndex(int index);
    bool Push(SensorModule *module);
    bool Delete(uint8_t id);
    int Count();
    
private:
    SensorModule *head;
    int count;
};
