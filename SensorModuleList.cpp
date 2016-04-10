#include "SensorModules.h"

SensorModuleList::SensorModuleList() {
    head = NULL;
    count = 0;
}

SensorModule *SensorModuleList::Get(uint8_t id) {
    SensorModule *ret = head;
    while(ret != NULL) {
        if(ret->address == id) {
            break;
        }
        ret = ret->next;
    }
    return ret;
}

SensorModule *SensorModuleList::GetByIndex(int index) {
  SensorModule *ret = head;
  if(!(index < count)) return NULL;

  for(int i = 0; i < index; i++) {
    // Advance the pointer
    ret = ret->next;
  }

  return ret;
}

bool SensorModuleList::Push(SensorModule *module) {
    SensorModule *end = head;
    
    // Cannot insert sensor module if its NULL
    if(module == NULL) return false;
    
    module->next = NULL;
    
    if(head == NULL) {
        head = module;
    }
    else {
        while(end->next != NULL) {
            if(end->address = module->address) {
                // Cannot have a duplicate canID
                return false;
            }
        }
        end->next = module;
    }
    count++;
    return true;
}

bool SensorModuleList::Delete(uint8_t id) {
    SensorModule *del = head, *prev = NULL;
    
    while(del != NULL) {
        if(del->address == id) {
            // Found the id, so delete this module
            if(prev == NULL) {
                // Change head to delete
                head = del->next;
            }
            else {
                // Delete in middle/end of list
                prev->next = del->next;
            }
            
            // Deallocate Sensor Module
            delete del;
            count--;
            return true;
        }
        
        prev = del;
    }
    
    // Not found
    return false;
}

int SensorModuleList::Count() {
  return count;
}

