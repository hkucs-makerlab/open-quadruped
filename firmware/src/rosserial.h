#pragma once

#include <ros/node_handle.h>

#include <BluetoothSerial.h>

namespace ros {
class BluetootSPP : public BluetoothSerial {
 public:
  void init() {
    uint64_t chipid = ESP.getEfuseMac();
    String id = String((uint16_t)(chipid >> 32), HEX);
    id.toUpperCase();
    begin("ROS-" + id);
    //Serial.println("Started, can pair it with bluetooth!");
  }
  void setBaud(int b = 115200) {}  // dummy function
  unsigned long time() { return millis(); }
};

class Serial2Hardware {
 public:
  Serial2Hardware() {
    iostream = &Serial2;
    baud_ = 38400;
  }

  void setBaud(long baud) { this->baud_ = baud; }

  int getBaud() { return baud_; }

  void init() { iostream->begin(baud_); }

  int read() { return iostream->read(); };

  void write(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      iostream->write(data[i]);
    }
  }

  unsigned long time() { return millis(); }

 protected:
  HardwareSerial* iostream;
  long baud_;
};

#define __ROS_USE_SPP_HANDLE__
#ifdef __ROS_USE_SPP_HANDLE__
typedef NodeHandle_<BluetootSPP, 5, 5, 512, 512> MyNodeHandle;
#else
typedef NodeHandle_<Serial2Hardware, 5, 5, 512, 512> MyNodeHandle;
#endif


}  // namespace ros
