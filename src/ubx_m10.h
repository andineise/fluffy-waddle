#pragma once

#include <Arduino.h>

// Minimal UBX parser for u-blox M10.
// Focus: NAV-PVT (class=0x01 id=0x07) and optional NAV-DOP (0x01 0x04)
// to generate NMEA GGA + RMC sentences.
class UbxM10 {
public:
  explicit UbxM10(HardwareSerial &serial);

  void begin(uint32_t baud, int8_t rxPin, int8_t txPin);

  // Polls serial, parses UBX frames.
  // Returns true if at least one full frame was processed.
  bool poll();

  // True if a new NAV-PVT fix was parsed since the last call.
  bool consumeNewFix();

  // Latest values (from NAV-PVT / NAV-DOP).
  bool hasFix() const;
  uint8_t numSats() const;

  String makeNmeaGga() const;
  String makeNmeaRmc() const;

  // Optional: configure module to output NAV-PVT + NAV-DOP over UART.
  // Deprecated: use configureUbx() instead.
  void sendBasicUbxConfig();

  // Configure module:
  // 1. Set UART to 460800, UBX In/Out only (disable NMEA).
  // 2. Set Rate to 20Hz.
  // 3. Enable NAV-PVT and NAV-DOP messages.
  void configureUbx();

  // Raw data accessors for RaceChrono DIY Protocol
  int32_t getLat1e7() const { return lat1e7_; }
  int32_t getLon1e7() const { return lon1e7_; }
  int32_t getAltMslMm() const { return hMSLmm_; }
  uint32_t getGSpeedMmS() const { return gSpeedMmS_; }
  int32_t getHeadMot1e5() const { return headMot1e5_; }
  uint8_t getFixType() const { return fixType_; }
  uint8_t getNumSats() const { return numSV_; }
  uint16_t getHdop1e2() const { return hDOP_1e2_; }
  uint16_t getVdop1e2() const { return vDOP_1e2_; }
  
  // Time
  uint16_t getYear() const { return year_; }
  uint8_t getMonth() const { return month_; }
  uint8_t getDay() const { return day_; }
  uint8_t getHour() const { return hour_; }
  uint8_t getMinute() const { return minute_; }
  uint8_t getSecond() const { return second_; }
  int32_t getNano() const { return nano_; }

private:
  HardwareSerial &serial_;
  
  // Baudrate-Erkennung
  bool tryBaudrate(uint32_t baud, int8_t rxPin, int8_t txPin);

  // Parser state.
  enum class State {
    Sync1,
    Sync2,
    Class,
    Id,
    Len1,
    Len2,
    Payload,
    CkA,
    CkB,
  };

  State state_ = State::Sync1;
  uint8_t msgClass_ = 0;
  uint8_t msgId_ = 0;
  uint16_t payloadLen_ = 0;
  uint16_t payloadPos_ = 0;
  uint8_t ckA_ = 0;
  uint8_t ckB_ = 0;
  uint8_t payload_[128] = {0};

  // Parsed NAV-PVT.
  bool newFix_ = false;
  bool gnssFixOk_ = false;
  uint8_t fixType_ = 0;
  uint8_t numSV_ = 0;
  int32_t lon1e7_ = 0;
  int32_t lat1e7_ = 0;
  int32_t hMSLmm_ = 0;
  uint32_t gSpeedMmS_ = 0;
  int32_t headMot1e5_ = 0;

  uint16_t year_ = 0;
  uint8_t month_ = 0;
  uint8_t day_ = 0;
  uint8_t hour_ = 0;
  uint8_t minute_ = 0;
  uint8_t second_ = 0;
  int32_t nano_ = 0;

  // Parsed NAV-DOP.
  uint16_t hDOP_1e2_ = 0; // 0.01
  uint16_t vDOP_1e2_ = 0; // 0.01

  void resetFrame_();
  void updateChecksum_(uint8_t byte);
  void handleFrame_();

  static uint16_t u16le_(const uint8_t *p);
  static uint32_t u32le_(const uint8_t *p);
  static int32_t i32le_(const uint8_t *p);

  static uint8_t nmeaChecksum_(const String &body);
  static String formatLatLon_(int32_t deg1e7, bool isLat, char &hemi);
  static String formatTime_(uint8_t hh, uint8_t mm, uint8_t ss);
  static String withChecksum_(const String &body);

  void sendUbx_(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);
};
