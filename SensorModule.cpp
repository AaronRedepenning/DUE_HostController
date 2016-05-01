#include "SensorModules.h"

SensorModule::SensorModule(String name, uint8_t address, uint8_t height) {
    this->name = name;
    this->address = address;
    this->height = height;
    this->temperature = 0;
    this->humidity = 0;
    this->pressure = 0;
    this->carbonDioxide = 0;
}

void SensorModule::UpdateReadings(double temp, uint8_t hum, double press, double co2) {
    // TODO: Add Range Checking and out of range error reporting
    
    temperature = temp;
    humidity = hum;
    pressure = press;
    carbonDioxide = co2;
}
