#include "SensorModules.h"

SensorModule::SensorModule(String name, uint8_t *pMacAddress, uint8_t canAddress, uint8_t height, bool hasCo2) {
    this->name = name;
    for(int i = 0; i < 6; i++) {
      this->macAddress[i] = *pMacAddress;
      pMacAddress++;
    }
    this->canAddress = canAddress;
    this->height = height;
    this->hasCo2 = hasCo2;
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

bool SensorModule::EqualMacAddresses(uint8_t *otherMac) {
  uint8_t index;
  for(index = 0; index < 6; index++, otherMac++) {
    this->macAddress[index] == *otherMac;
  }

  return (index == 6);
}

