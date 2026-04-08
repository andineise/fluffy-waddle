#include "ble_nmea_uart.h"
#include "ubx_m10.h"

#include <NimBLEDevice.h>

namespace {

// =============================================================
// RaceChrono DIY Protocol (Pro Mode)
// =============================================================
// Use full 128-bit UUIDs based on Bluetooth Base UUID
// Service UUID: 0x1FF8
static const NimBLEUUID kRcServiceUuid("00001ff8-0000-1000-8000-00805f9b34fb");
// CAN-Bus Main Characteristic (0x0001) - Read/Notify - Data from device to app
static const NimBLEUUID kRcCanUuid("00000001-0000-1000-8000-00805f9b34fb");
// PID Request Characteristic (0x0002) - Write - App tells device which PIDs to send
static const NimBLEUUID kRcPidRequestUuid("00000002-0000-1000-8000-00805f9b34fb");
// GPS Main Characteristic (0x0003) - Read/Notify  
static const NimBLEUUID kRcGpsUuid("00000003-0000-1000-8000-00805f9b34fb");

// =============================================================
// Nordic UART Service (NMEA Mode)
// =============================================================
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const NimBLEUUID kNusServiceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
// RX (App -> Device): 6E400002...
static const NimBLEUUID kNusRxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
// TX (Device -> App): 6E400003...
static const NimBLEUUID kNusTxUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

class ServerCallbacks : public NimBLEServerCallbacks {
public:
  explicit ServerCallbacks(bool *connected) : connected_(connected) {}

  void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
    *connected_ = true;
    Serial.println("[BLE] Device connected!");
    
    // Request fast connection parameters for high update rate
    // minInterval: 7.5ms (6 * 1.25ms), maxInterval: 15ms (12 * 1.25ms)
    // latency: 0 (respond every interval), timeout: 200 (2000ms)
    server->updateConnParams(connInfo.getConnHandle(), 6, 12, 0, 200);
    Serial.println("[BLE] Requested fast connection params: 7.5-15ms interval");
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo & /*connInfo*/, int /*reason*/) override {
    *connected_ = false;
    Serial.println("[BLE] Device disconnected, restarting advertising...");
    NimBLEDevice::startAdvertising();
    (void)server;
  }

private:
  bool *connected_;
};

// Callback for PID Request characteristic - RaceChrono tells us which PIDs to send
class PidRequestCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic *chr, NimBLEConnInfo & /*connInfo*/) override {
    NimBLEAttValue value = chr->getValue();
    const uint8_t *data = value.data();
    uint16_t len = value.length();
    
    Serial.print("[RC] PID Request received, len=");
    Serial.print(len);
    Serial.print(" data: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", data[i]);
    }
    Serial.println();
    
    // Protocol:
    // data[0] = command type
    //   0 = deny all PIDs
    //   1 = allow all PIDs, data[1..2] = update interval ms (big endian)
    //   2 = allow one PID, data[1..2] = interval, data[3..6] = PID (big endian)
    if (len >= 1) {
      switch (data[0]) {
        case 0:
          Serial.println("[RC] Command: DENY ALL PIDS");
          break;
        case 1:
          if (len >= 3) {
            uint16_t interval = (data[1] << 8) | data[2];
            Serial.printf("[RC] Command: ALLOW ALL PIDS, interval=%dms\n", interval);
          }
          break;
        case 2:
          if (len >= 7) {
            uint16_t interval = (data[1] << 8) | data[2];
            uint32_t pid = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6];
            Serial.printf("[RC] Command: ALLOW PID 0x%X, interval=%dms\n", pid, interval);
          }
          break;
      }
    }
  }
};

class NusRxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic * /*chr*/, NimBLEConnInfo & /*connInfo*/) override {
    // Ignored in NMEA mode
  }
};

// Callback to track when RaceChrono subscribes to GPS notifications
class GpsCharCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onSubscribe(NimBLECharacteristic *chr, NimBLEConnInfo &connInfo, uint16_t subValue) override {
    Serial.printf("[BLE] Char %s subscription changed: %s (connHandle=%d)\n",
                  chr->getUUID().toString().c_str(),
                  subValue == 0 ? "UNSUBSCRIBED" : (subValue == 1 ? "NOTIFY" : "INDICATE"),
                  connInfo.getConnHandle());
  }
  
  void onRead(NimBLECharacteristic *chr, NimBLEConnInfo & /*connInfo*/) override {
    Serial.printf("[BLE] Char %s was READ (polling, not notify)\n", chr->getUUID().toString().c_str());
  }
};

} // namespace

// RaceChrono DIY uses a special 128-bit UUID for the service
// From official example: 0x00000001000000fd8933990d6f411ff8
// Which is: 1ff8-1146-0d99-3389-fd00-000001000000 (reversed for BLE)
// Simplified: use the documented 16-bit UUIDs
static const uint16_t RC_SERVICE_UUID      = 0x1FF8;
static const uint16_t RC_CAN_CHAR_UUID     = 0x0001;
static const uint16_t RC_PID_REQ_CHAR_UUID = 0x0002;
static const uint16_t RC_GPS_CHAR_UUID     = 0x0003;
static const uint16_t RC_GPS_TIME_CHAR_UUID= 0x0004;

void BleNmeaUart::begin(const String &deviceName, bool proMode) {
  proMode_ = proMode;

  Serial.println("[BLE] Initializing...");
  Serial.print("[BLE] Device name: ");
  Serial.println(deviceName);
  Serial.print("[BLE] Pro mode: ");
  Serial.println(proMode ? "YES" : "NO (NMEA)");

  NimBLEDevice::init(deviceName.c_str());
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks(&connected_));
  
  Serial.println("[BLE] Server created");

  if (proMode_) {
    // ---------------------------------------------------------
    // Init RaceChrono Pro Service (0x1FF8)
    // Matches official arduino-RaceChrono library
    // ---------------------------------------------------------
    NimBLEService *service = server->createService(RC_SERVICE_UUID);
    
    // Callback instance for tracking subscriptions
    static GpsCharCallbacks gpsCallbacks;

    // PID Request Char (0x0002) - Write - App tells device which PIDs to send
    NimBLECharacteristic *pidReqChar = service->createCharacteristic(
        RC_PID_REQ_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
    pidReqChar->setCallbacks(new PidRequestCallbacks());

    // CAN Bus Data Char (0x0001) - Read/Notify - Device sends CAN data to app
    NimBLECharacteristic *canChar = service->createCharacteristic(
        RC_CAN_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    canChar->setCallbacks(&gpsCallbacks);
    rcCanChar_ = canChar;

    // GPS Main Char (0x0003) - Read/Notify - Main GPS data
    NimBLECharacteristic *gpsChar = service->createCharacteristic(
        RC_GPS_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    gpsChar->setCallbacks(&gpsCallbacks);
    rcGpsChar_ = gpsChar;

    // GPS Time Char (0x0004) - Read/Notify - GPS time/date (REQUIRED!)
    NimBLECharacteristic *gpsTimeChar = service->createCharacteristic(
        RC_GPS_TIME_CHAR_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    gpsTimeChar->setCallbacks(&gpsCallbacks);
    rcGpsTimeChar_ = gpsTimeChar;

    service->start();
    
    Serial.println("[BLE] RaceChrono DIY service started (0x1FF8)");
    Serial.printf("[BLE]   Service UUID: %s\n", service->getUUID().toString().c_str());
    Serial.println("[BLE]   - PID Request Char 0x0002 (Write)");
    Serial.println("[BLE]   - CAN Data Char 0x0001 (Notify)");
    Serial.println("[BLE]   - GPS Main Char 0x0003 (Notify)");
    Serial.println("[BLE]   - GPS Time Char 0x0004 (Notify)");

    // Setup advertising - match official library
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setMinInterval(32);   // 20ms
    adv->setMaxInterval(160);  // 100ms
    adv->addServiceUUID(service->getUUID());
    adv->start();
    
    Serial.println("[BLE] Advertising started!");

  } else {
    // ---------------------------------------------------------
    // Nordic UART Service (NUS) mit NMEA Sentences
    // Das ist der Standard für BLE GPS Logger wie z.B. Bad Elf
    // RaceChrono erkennt das als "Bluetooth GPS"
    // ---------------------------------------------------------
    
    NimBLEService *service = server->createService(kNusServiceUuid);

    // TX Characteristic (Device -> App) - hier senden wir NMEA
    nusTxChar_ = service->createCharacteristic(
        kNusTxUuid, NIMBLE_PROPERTY::NOTIFY);

    // RX Characteristic (App -> Device) - für Kommandos
    NimBLECharacteristic *rxChar = service->createCharacteristic(
        kNusRxUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxChar->setCallbacks(new NusRxCallbacks());

    service->start();

    // Setup advertising
    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advData;
    advData.setFlags(0x06);  // LE General Discoverable, BR/EDR Not Supported
    advData.setCompleteServices(kNusServiceUuid);
    adv->setAdvertisementData(advData);

    NimBLEAdvertisementData scanResp;
    scanResp.setName(deviceName.c_str());
    adv->setScanResponseData(scanResp);

    adv->start();
    Serial.println("[BLE] Nordic UART Service (NMEA GPS) started!");
    Serial.println("[BLE]   - TX Char (Notify): 6E400003...");
    Serial.println("[BLE]   - RX Char (Write):  6E400002...");
  }

  server_ = server;
  Serial.print("[BLE] Address: ");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());
  syncCounter_ = 0;
}

bool BleNmeaUart::isConnected() const {
  return connected_;
}

// -------------------------------------------------------------------------
// NMEA over BLE Nordic UART Service
// -------------------------------------------------------------------------

// Send NMEA sentence over BLE
void BleNmeaUart::sendNmea(const String &line) {
  if (!connected_ || !nusTxChar_ || proMode_) return;
  
  auto *tx = static_cast<NimBLECharacteristic *>(nusTxChar_);
  
  // NMEA sentence muss mit \r\n enden
  String nmea = line;
  if (!nmea.endsWith("\r\n")) {
    nmea += "\r\n";
  }
  
  // BLE MTU ist typisch 20 Bytes, aber wir können größere Pakete senden
  // NimBLE fragmentiert automatisch
  tx->setValue((uint8_t*)nmea.c_str(), nmea.length());
  tx->notify();
}

// Send GPS data as NMEA sentences (GGA + RMC)
void BleNmeaUart::sendLocationSpeed(const UbxM10& gps) {
  static uint32_t callCount = 0;
  callCount++;
  
  if (!connected_) {
    return;
  }
  
  if (!nusTxChar_) {
    Serial.println("[NMEA] ERROR: nusTxChar_ is NULL!");
    return;
  }
  
  if (proMode_) {
    return;
  }

  // Generiere und sende NMEA Sentences
  String gga = gps.makeNmeaGga();
  String rmc = gps.makeNmeaRmc();
  
  sendNmea(gga);
  sendNmea(rmc);
  
  // Debug alle 10 Aufrufe
  if (callCount % 10 == 1) {
    Serial.printf("[NMEA TX #%d] sats=%d, fix=%d\n", callCount, gps.getNumSats(), gps.hasFix());
    Serial.println(gga);
  }
}

// -------------------------------------------------------------------------
// PRO MODE Helpers
// -------------------------------------------------------------------------
void BleNmeaUart::sendCanData(uint32_t id, const uint8_t* data, uint8_t len) {
    if (!connected_ || !rcCanChar_ || !proMode_) return;
    if (len > 16) return; // Spec says max 16 bytes payload

    // Format: PID (4 bytes LE) + Payload (N bytes)
    uint8_t buf[20];
    buf[0] = id & 0xFF;
    buf[1] = (id >> 8) & 0xFF;
    buf[2] = (id >> 16) & 0xFF;
    buf[3] = (id >> 24) & 0xFF;

    memcpy(&buf[4], data, len);

    auto *ch = static_cast<NimBLECharacteristic *>(rcCanChar_);
    ch->setValue(buf, 4 + len);
    ch->notify();
}

void BleNmeaUart::updateGps(const UbxM10& gps) {
  if (!connected_ || !rcGpsChar_ || !proMode_) return;

  // Debug: Zähle wie oft diese Funktion aufgerufen wird (alle 30s)
  static uint32_t callCount = 0;
  static uint32_t lastReport = 0;
  callCount++;
  if (millis() - lastReport > 30000) {
    Serial.printf("[BLE] %.1f Hz (%lu/30s), fix=%d, sats=%d, lat=%.6f\n", 
                  callCount/30.0f, callCount, gps.getFixType(), gps.getNumSats(), gps.getLat1e7()/1e7);
    callCount = 0;
    lastReport = millis();
  }

  uint8_t buf[20];
  
  // 1. Time & Sync (3 bytes)
  // Time from hour start = (min * 30000) + (sec * 500) + (ms / 2)
  long ms = gps.getNano() / 1000000;
  if (ms < 0) ms = 0;
  uint32_t timeVal = (gps.getMinute() * 30000) + (gps.getSecond() * 500) + (ms / 2);
  
  // 3-bit Sync counter (increments when time char changes)
  static uint8_t gpsPreviousDateAndHour = 0;
  uint16_t year = gps.getYear();
  uint8_t month = gps.getMonth();
  uint8_t day = gps.getDay();
  uint8_t hour = gps.getHour();
  
  // Calculate dateAndHour for time characteristic
  uint32_t dateAndHour = 0;
  if (year >= 2000) {
    dateAndHour = ((year - 2000) * 8928) + ((month - 1) * 744) + ((day - 1) * 24) + hour;
  }
  
  // Increment sync when dateAndHour changes (like official implementation)
  if (gpsPreviousDateAndHour != (dateAndHour & 0xFF)) {
    gpsPreviousDateAndHour = dateAndHour & 0xFF;
    syncCounter_++;
  }
  uint8_t sync = syncCounter_ & 0x07;

  // Byte 0-2: Sync(3 bits) | Time from hour start (21 bits)
  // EXACT format from official example:
  // tempData[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
  buf[0] = ((sync & 0x07) << 5) | ((timeVal >> 16) & 0x1F);
  buf[1] = (timeVal >> 8) & 0xFF;
  buf[2] = timeVal & 0xFF;

  // 2. Fix Quality & Sats (1 byte)
  // Fix(2) | Sats(6)
  // RaceChrono expects NMEA GGA fix quality:
  // 0 = Invalid, 1 = GPS fix, 2 = DGPS, 3 = PPS/RTK
  // UBX fixType: 0=No fix, 2=2D, 3=3D, 4=GNSS+DR, 5=Time only
  uint8_t fix = 0;
  uint8_t ft = gps.getFixType();
  if (ft >= 2) fix = 1;      // 2D or 3D -> GPS fix (quality 1)
  // Could use fix = 2 for SBAS/DGPS if we detect it
  
  uint8_t sats = gps.getNumSats();
  if (sats > 63) sats = 63;
  buf[3] = (fix << 6) | (sats & 0x3F);

  // 3. Latitude (4 bytes, Big Endian)
  int32_t lat = gps.getLat1e7();
  buf[4] = (lat >> 24) & 0xFF;
  buf[5] = (lat >> 16) & 0xFF;
  buf[6] = (lat >> 8) & 0xFF;
  buf[7] = (lat) & 0xFF;

  // 4. Longitude (4 bytes, Big Endian)
  int32_t lon = gps.getLon1e7();
  buf[8] = (lon >> 24) & 0xFF;
  buf[9] = (lon >> 16) & 0xFF;
  buf[10] = (lon >> 8) & 0xFF;
  buf[11] = (lon) & 0xFF;

  // 5. Altitude (2 bytes, Big Endian)
  double altMeters = gps.getAltMslMm() / 1000.0;
  int16_t altEnc = 0;
  // Format 1: ((meters + 500) * 10) & 0x7FFF
  // Range: -500 to +6053.5m
  if (altMeters >= -500.0 && altMeters <= 6053.5) {
      altEnc = (int16_t)((altMeters + 500.0) * 10.0) & 0x7FFF;
  } else {
      // Format 2: ((meters + 500) & 0x7FFF) | 0x8000
      altEnc = ((int16_t)(altMeters + 500.0) & 0x7FFF) | 0x8000;
  }
  buf[12] = (altEnc >> 8) & 0xFF;
  buf[13] = (altEnc) & 0xFF;

  // 6. Speed (2 bytes, Big Endian)
  // km/h
  double speedKmh = gps.getGSpeedMmS() * 0.0036;
  int16_t speedEnc = 0;
  // Format 1: (km/h * 100) & 0x7FFF -> Max 655.35 km/h
  if (speedKmh <= 655.35) {
      speedEnc = (int16_t)(speedKmh * 100.0) & 0x7FFF;
  } else {
      // Format 2: (km/h * 10) & 0x7FFF | 0x8000
      speedEnc = ((int16_t)(speedKmh * 10.0) & 0x7FFF) | 0x8000;
  }
  buf[14] = (speedEnc >> 8) & 0xFF;
  buf[15] = (speedEnc) & 0xFF;

  // 7. Bearing (2 bytes, Big Endian)
  // degrees * 100
  uint16_t bearEnc = (uint16_t)(gps.getHeadMot1e5() / 1000); 
  buf[16] = (bearEnc >> 8) & 0xFF;
  buf[17] = (bearEnc) & 0xFF;

  // 8. HDOP (1 byte)
  // dop * 10. UBX hDOP is 1e-2 units.
  uint8_t hdopEnc = (uint8_t)(gps.getHdop1e2() / 10);
  buf[18] = hdopEnc;

  // 9. VDOP (1 byte)
  uint8_t vdopEnc = (uint8_t)(gps.getVdop1e2() / 10);
  buf[19] = vdopEnc;

  auto *tx = static_cast<NimBLECharacteristic *>(rcGpsChar_);
  tx->setValue(buf, 20);
  tx->notify();  // NimBLE handles subscription check internally

  // GPS Time Characteristic (0x0004) - Set value for READ/polling
  // According to docs: "The app reads (polls) this characteristic when needed"
  if (rcGpsTimeChar_) {
    uint8_t timeBuf[3];
    
    timeBuf[0] = ((sync & 0x07) << 5) | ((dateAndHour >> 16) & 0x1F);
    timeBuf[1] = (dateAndHour >> 8) & 0xFF;
    timeBuf[2] = dateAndHour & 0xFF;
    
    auto *timeChar = static_cast<NimBLECharacteristic *>(rcGpsTimeChar_);
    timeChar->setValue(timeBuf, 3);
    timeChar->notify();  // Notify for apps that subscribe
  }
  
}

int BleNmeaUart::getGpsSubscriberCount() const {
  return connected_ ? 1 : 0;  // Simplified - NimBLE 2.3 doesn't expose subscriber count
}

int BleNmeaUart::getTimeSubscriberCount() const {
  return connected_ ? 1 : 0;
}
