#pragma once

#include <Arduino.h>

class UbxM10;
class BluetoothSerial;  // Forward declaration

// Dual-Mode Bluetooth Handler:
// Mode false: BLE Location & Navigation Service (0x1819) - for RaceChrono "Bluetooth Smart LN"
// Mode true:  BLE RaceChrono DIY Protocol (0x1FF8) with High-Freq GPS + CAN Bus (Pro required)
class BleNmeaUart {
public:
  void begin(const String &deviceName, bool proMode);
  bool isConnected() const;

  // Mode: BLE Location and Navigation (Free) - sends binary GPS data
  void sendNmea(const String &line);  // Legacy - now calls sendLocationSpeed internally
  void sendLocationSpeed(const UbxM10& gps);  // New - sends BLE LN format

  // Mode: BLE Pro (0x1FF8)
  void updateGps(const UbxM10& gps);
  void sendCanData(uint32_t id, const uint8_t* data, uint8_t len);
  
  // Debug: Check subscriber count
  int getGpsSubscriberCount() const;
  int getTimeSubscriberCount() const;

private:
  bool connected_ = false;
  bool proMode_ = false;
  
  // Pro Mode (BLE RaceChrono DIY)
  void *server_ = nullptr;
  void *rcGpsChar_ = nullptr;
  void *rcGpsTimeChar_ = nullptr;  // GPS Time Characteristic (0x0004)
  void *rcCanChar_ = nullptr;
  uint8_t syncCounter_ = 0;

  // LN Mode (BLE Location and Navigation Service)
  void *nusTxChar_ = nullptr;  // Location and Speed Characteristic
  void *posQualityChar_ = nullptr;  // Position Quality Characteristic
};
