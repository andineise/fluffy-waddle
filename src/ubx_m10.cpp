#include "ubx_m10.h"

UbxM10::UbxM10(HardwareSerial &serial) : serial_(serial) {}

// Versucht bei gegebener Baudrate UBX-Pakete zu empfangen
bool UbxM10::tryBaudrate(uint32_t baud, int8_t rxPin, int8_t txPin) {
  serial_.end();
  delay(10);
  serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
  delay(100); // Warten bis UART stabil
  
  // Buffer leeren
  unsigned long clearStart = millis();
  while(serial_.available() && (millis() - clearStart < 100)) {
    serial_.read();
  }
  
  // UBX Poll-Request für MON-VER senden (funktioniert immer)
  // 0xB5 0x62 0x0A 0x04 0x00 0x00 0x0E 0x34
  uint8_t pollMonVer[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
  serial_.write(pollMonVer, sizeof(pollMonVer));
  serial_.flush();
  
  // Warte auf Antwort (max 500ms)
  unsigned long start = millis();
  int syncCount = 0;
  uint8_t lastByte = 0;
  
  while(millis() - start < 500) {
    if(serial_.available()) {
      uint8_t b = serial_.read();
      
      // Suche nach UBX Sync-Sequenz 0xB5 0x62
      if(lastByte == 0xB5 && b == 0x62) {
        syncCount++;
        if(syncCount >= 1) {
          Serial.printf("[GPS] Baudrate %lu erkannt (UBX Sync gefunden)\n", baud);
          return true;
        }
      }
      lastByte = b;
    }
  }
  
  Serial.printf("[GPS] Keine Antwort bei %lu baud\n", baud);
  return false;
}

void UbxM10::begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
  Serial.println("[GPS] Starte Baudrate-Erkennung...");
  
  // Typische u-blox Baudraten (häufigste zuerst)
  const uint32_t baudrates[] = {
    460800,  // Unsere Ziel-Baudrate (falls schon konfiguriert)
    38400,   // u-blox Default
    115200,  // Häufig verwendet
    9600,    // Sehr alter Default
    57600,
    230400
  };
  
  uint32_t detectedBaud = 0;
  
  for(size_t i = 0; i < sizeof(baudrates)/sizeof(baudrates[0]); i++) {
    if(tryBaudrate(baudrates[i], rxPin, txPin)) {
      detectedBaud = baudrates[i];
      break;
    }
  }
  
  if(detectedBaud == 0) {
    // Fallback: Nimm die übergebene Baudrate
    Serial.printf("[GPS] Keine Baudrate erkannt, verwende %lu als Fallback\n", baud);
    serial_.end();
    delay(10);
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
    detectedBaud = baud;
  }
  
  Serial.printf("[GPS] Aktive Baudrate: %lu\n", detectedBaud);
  
  // Buffer leeren vor Konfiguration
  delay(50);
  unsigned long clearStart = millis();
  while(serial_.available() && (millis() - clearStart < 200)) {
    serial_.read();
  }
}

// -------------------------------------------------------------------------
//  Improved poll() - Returns true ONLY when a new NAV-PVT fix is READY.
// -------------------------------------------------------------------------
bool UbxM10::poll() {
  bool fixReady = false;
  int limit = 512; // Process up to 512 bytes per slice to prevent task starvation

  while (serial_.available() > 0 && limit-- > 0) {
    uint8_t b = static_cast<uint8_t>(serial_.read());

    switch (state_) {
      case State::Sync1:
        if (b == 0xB5) {
          resetFrame_();
          state_ = State::Sync2;
        }
        break;

      case State::Sync2:
        if (b == 0x62) {
          state_ = State::Class;
        } else {
          state_ = State::Sync1;
        }
        break;

      case State::Class:
        msgClass_ = b;
        updateChecksum_(b);
        state_ = State::Id;
        break;

      case State::Id:
        msgId_ = b;
        updateChecksum_(b);
        state_ = State::Len1;
        break;

      case State::Len1:
        payloadLen_ = b;
        updateChecksum_(b);
        state_ = State::Len2;
        break;

      case State::Len2:
        payloadLen_ |= static_cast<uint16_t>(b) << 8;
        updateChecksum_(b);
        payloadPos_ = 0;
        if (payloadLen_ > sizeof(payload_)) {
          // Oversized frame, drop.
          state_ = State::Sync1;
        } else if (payloadLen_ == 0) {
          state_ = State::CkA;
        } else {
          state_ = State::Payload;
        }
        break;

      case State::Payload:
        payload_[payloadPos_++] = b;
        updateChecksum_(b);
        if (payloadPos_ >= payloadLen_) {
          state_ = State::CkA;
        }
        break;

      case State::CkA:
        if (b == ckA_) {
          state_ = State::CkB;
        } else {
          state_ = State::Sync1;
        }
        break;

      case State::CkB:
        if (b == ckB_) {
          // Frame valid!
          handleFrame_();
          
          // Debug: Zeige NAV-PVT Pakete
          if (msgClass_ == 0x01 && msgId_ == 0x07) {
            static uint32_t pvtCount = 0;
            pvtCount++;
            if (pvtCount % 100 == 0) {
              Serial.printf("[UBX] NAV-PVT #%lu: fix=%d, sats=%d, lat=%.6f, lon=%.6f\n", 
                            pvtCount, fixType_, numSV_, lat1e7_/1e7, lon1e7_/1e7);
            }
          }
          
          // Only trigger "processed" if it was a NAV-PVT (0x01/0x07)
          // This ensures we sync BLE updates to the Navigation Solution
          if (newFix_) {
              fixReady = true;
          }
          
          state_ = State::Sync1;
        } else {
            state_ = State::Sync1;
        }
        break;
    }
  }

  return fixReady;
}

// -------------------------------------------------------------------------
//  Fix Consumption - Simply returns the newFix flag without clearing it
//  (clearing happens in poll when next fix arrives, actually handled via poll return)
// -------------------------------------------------------------------------
bool UbxM10::consumeNewFix() {
    bool ret = newFix_;
    if (ret) newFix_ = false; // Clear flag
    return ret;
}

bool UbxM10::hasFix() const {
  // UBX NAV-PVT fixType: 3=3D fix, 4=GNSS+DR, 5=Time only.
  return gnssFixOk_ && (fixType_ >= 3);
}

uint8_t UbxM10::numSats() const {
  return numSV_;
}

String UbxM10::makeNmeaGga() const {
  // $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,,
  const String timeStr = formatTime_(hour_, minute_, second_);

  char latHemi = 'N';
  char lonHemi = 'E';
  const String latStr = formatLatLon_(lat1e7_, true, latHemi);
  const String lonStr = formatLatLon_(lon1e7_, false, lonHemi);

  const int fixQuality = hasFix() ? 1 : 0;
  const uint8_t sats = numSV_;
  const float hdop = (hDOP_1e2_ > 0) ? (hDOP_1e2_ / 100.0f) : 0.9f;
  const float altMeters = hMSLmm_ / 1000.0f;

  String body = "GPGGA,";
  body += timeStr;
  body += ",";
  body += latStr;
  body += ",";
  body += latHemi;
  body += ",";
  body += lonStr;
  body += ",";
  body += lonHemi;
  body += ",";
  body += String(fixQuality);
  body += ",";
  body += String(sats);
  body += ",";
  body += String(hdop, 1);
  body += ",";
  body += String(altMeters, 1);
  body += ",M,,M,,";

  return withChecksum_(body);
}

String UbxM10::makeNmeaRmc() const {
  // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,speed,course,ddmmyy,,,A
  const String timeStr = formatTime_(hour_, minute_, second_);

  char latHemi = 'N';
  char lonHemi = 'E';
  const String latStr = formatLatLon_(lat1e7_, true, latHemi);
  const String lonStr = formatLatLon_(lon1e7_, false, lonHemi);

  const char status = hasFix() ? 'A' : 'V';

  const float speedMs = gSpeedMmS_ / 1000.0f;
  const float speedKnots = speedMs * 1.943844f;
  const float courseDeg = headMot1e5_ / 100000.0f;

  char dateBuf[7];
  // ddmmyy
  snprintf(dateBuf, sizeof(dateBuf), "%02u%02u%02u", day_, month_, static_cast<uint8_t>(year_ % 100));

  String body = "GPRMC,";
  body += timeStr;
  body += ",";
  body += status;
  body += ",";
  body += latStr;
  body += ",";
  body += latHemi;
  body += ",";
  body += lonStr;
  body += ",";
  body += lonHemi;
  body += ",";
  body += String(speedKnots, 1);
  body += ",";
  body += String(courseDeg, 1);
  body += ",";
  body += dateBuf;
  body += ",,,A";

  return withChecksum_(body);
}

void UbxM10::sendBasicUbxConfig() {
  // Enable NAV-PVT (0x01 0x07) and NAV-DOP (0x01 0x04) on UART1 at 1 Hz+.
  // UBX-CFG-MSG: class=0x06 id=0x01, payload: msgClass,msgId,rateUART1
  // Many u-blox chips use a 8-byte payload (rates for I2C/UART1/UART2/USB/SPI),
  // but M10 supports 3-byte legacy as well for some configs. We'll use 8-byte
  // variant for compatibility: rates per port.
  auto sendCfgMsg = [&](uint8_t cls, uint8_t id, uint8_t rateUart1) {
    uint8_t p[8] = {cls, id, 0, rateUart1, 0, 0, 0, 0};
    sendUbx_(0x06, 0x01, p, sizeof(p));
  };

  // 10 Hz rate: UBX-CFG-RATE (0x06 0x08), payload: measRate(ms), navRate, timeRef.
  // measRate=100ms, navRate=1, timeRef=1 (GPS time).
  {
    uint8_t p[6] = {0x64, 0x00, 0x01, 0x00, 0x01, 0x00};
    sendUbx_(0x06, 0x08, p, sizeof(p));
  }

  sendCfgMsg(0x01, 0x07, 1); // NAV-PVT
  sendCfgMsg(0x01, 0x04, 1); // NAV-DOP
}

void UbxM10::configureUbx() {
  Serial.println("[GPS] ========== Starte UBX M10 Konfiguration ==========");
  
  // WICHTIG: M10 nutzt das CFG-VALSET (0x06 0x8A) Protokoll.
  // KEIN Factory Reset - das würde die Baudrate ändern und Kommunikation unterbrechen!
  // Wir setzen nur die Werte die wir brauchen.
  //
  // Layer-Kombinationen für CFG-VALSET:
  // 0x01 = RAM only (flüchtig, sofort aktiv)
  // 0x02 = BBR (Battery Backed RAM - bleibt bei Neustart)
  // 0x04 = Flash (permanent)
  // 0x07 = RAM + BBR + Flash (alle drei)
  //
  // Wir nutzen Layer 0x07 für permanente Speicherung!
  
  const uint8_t LAYER_ALL = 0x07;  // RAM + BBR + Flash
  
  // ---------------------------------------------------------------------------------
  // SCHRITT 1: Messrate auf 20Hz (50ms)
  // ---------------------------------------------------------------------------------
  Serial.println("[GPS CFG] Schritt 1: Messrate 50ms (20Hz) -> RAM+BBR+Flash...");
  {
    // CFG-RATE-MEAS (0x30210001) -> 50ms
    uint8_t cfgRate[] = {
      0x00,                 // Version
      LAYER_ALL,            // Layer: RAM + BBR + Flash
      0x00, 0x00,           // Reserved
      
      // Key: CFG-RATE-MEAS = 50ms (20Hz)
      0x01, 0x00, 0x21, 0x30, // Key ID (0x30210001)
      0x32, 0x00              // Value: 50 (U2, little endian)
    };
    sendUbx_(0x06, 0x8A, cfgRate, sizeof(cfgRate));
    delay(100);
  }

  // ---------------------------------------------------------------------------------
  // SCHRITT 2: GNSS Konstellationen (GPS + Galileo ONLY) - für 20Hz nötig
  // ---------------------------------------------------------------------------------
  Serial.println("[GPS CFG] Schritt 2: GNSS Konstellationen (GPS+Galileo) -> RAM+BBR+Flash...");
  {
    uint8_t cfgConst[] = {
      0x00, LAYER_ALL, 0x00, 0x00, // Header (Layer RAM+BBR+Flash)
      
      // GPS Enable = 1
      0x1F, 0x00, 0x31, 0x10, 0x01,
      // Galileo Enable = 1
      0x21, 0x00, 0x31, 0x10, 0x01,
      // Glonass Enable = 0
      0x25, 0x00, 0x31, 0x10, 0x00,
      // BeiDou Enable = 0
      0x22, 0x00, 0x31, 0x10, 0x00
    };
    sendUbx_(0x06, 0x8A, cfgConst, sizeof(cfgConst));
    delay(100);
  }

  // ---------------------------------------------------------------------------------
  // SCHRITT 3: Automotive Dynamic Model
  // ---------------------------------------------------------------------------------
  Serial.println("[GPS CFG] Schritt 3: Automotive Mode -> RAM+BBR+Flash...");
  {
    uint8_t cfgDyn[] = {
      0x00, LAYER_ALL, 0x00, 0x00, // Header (RAM+BBR+Flash)
      // CFG-NAVSPG-DYNMODEL = 4 (Automotive)
      0x21, 0x00, 0x11, 0x20, 0x04
    };
    sendUbx_(0x06, 0x8A, cfgDyn, sizeof(cfgDyn));
    delay(100);
  }

  // ---------------------------------------------------------------------------------
  // SCHRITT 4: ALLE Nachrichten AUS, dann NUR NAV-PVT AN
  // ---------------------------------------------------------------------------------
  Serial.println("[GPS CFG] Schritt 4: Deaktiviere ALLE Nachrichten -> RAM+BBR+Flash...");
  {
    // Alle störenden Nachrichten explizit deaktivieren
    uint8_t cfgMsgOff[] = {
      0x00, LAYER_ALL, 0x00, 0x00, // Header (RAM+BBR+Flash)
      
      // NAV-DOP aus (0x20910039)
      0x39, 0x00, 0x91, 0x20, 0x00,
      // NAV-POSLLH aus (0x20910030) 
      0x30, 0x00, 0x91, 0x20, 0x00,
      // NAV-STATUS aus (0x2091001B)
      0x1B, 0x00, 0x91, 0x20, 0x00,
      // NAV-VELNED aus (0x20910044)
      0x44, 0x00, 0x91, 0x20, 0x00,
      // NAV-TIMEUTC aus (0x2091005C)
      0x5C, 0x00, 0x91, 0x20, 0x00
    };
    sendUbx_(0x06, 0x8A, cfgMsgOff, sizeof(cfgMsgOff));
    delay(100);
  }
  
  {
    // Weitere Nachrichten deaktivieren (zweiter Block wegen Größenlimit)
    uint8_t cfgMsgOff2[] = {
      0x00, LAYER_ALL, 0x00, 0x00, // RAM+BBR+Flash
      // NAV-SAT aus (0x20910016)
      0x16, 0x00, 0x91, 0x20, 0x00,
      // NAV-ORB aus (0x20910011)
      0x11, 0x00, 0x91, 0x20, 0x00,
      // NAV-CLOCK aus (0x20910066)
      0x66, 0x00, 0x91, 0x20, 0x00,
      // NAV-AOPSTATUS aus (0x20910080)
      0x80, 0x00, 0x91, 0x20, 0x00,
      // NAV-SIG aus (0x20910346)
      0x46, 0x03, 0x91, 0x20, 0x00
    };
    sendUbx_(0x06, 0x8A, cfgMsgOff2, sizeof(cfgMsgOff2));
    delay(100);
  }
  
  Serial.println("[GPS CFG] Schritt 5: Aktiviere NUR NAV-PVT -> RAM+BBR+Flash...");
  {
    // NAV-PVT aktivieren mit Rate 1 (jeden Messzyklus)
    uint8_t cfgPvt[] = {
      0x00, LAYER_ALL, 0x00, 0x00, // RAM+BBR+Flash
      // NAV-PVT UART1 = 1 (0x20910007)
      0x07, 0x00, 0x91, 0x20, 0x01,
      // NMEA Protokoll aus
      0x02, 0x00, 0x74, 0x10, 0x00
    };
    sendUbx_(0x06, 0x8A, cfgPvt, sizeof(cfgPvt));
    delay(100);
  }

  // ---------------------------------------------------------------------------------
  // SCHRITT 6: Baudrate - NUR wenn nicht schon auf 460800
  // ---------------------------------------------------------------------------------
  // TEMPORÄR DEAKTIVIERT für Debugging - erstmal ohne Baudrate-Änderung testen
  /*
  Serial.println("[GPS CFG] Schritt 6: Baudrate auf 460800 -> RAM+BBR+Flash...");
  {
    uint8_t cfgBaud[] = {
      0x00, LAYER_ALL, 0x00, 0x00, // Header (RAM+BBR+Flash)
      // CFG-UART1-BAUDRATE (0x40520001) -> 460800
      0x01, 0x00, 0x52, 0x40,
      0x00, 0x08, 0x07, 0x00  // Value: 460800 (little endian)
    };
    sendUbx_(0x06, 0x8A, cfgBaud, sizeof(cfgBaud));
    
    delay(100);
    
    // Buffer leeren
    unsigned long startClear = millis();
    while(serial_.available() && (millis() - startClear < 100)) {
       serial_.read();
    }

    serial_.flush();
    serial_.updateBaudRate(460800);

    delay(100);
    startClear = millis();
    while(serial_.available() && (millis() - startClear < 100)) {
       serial_.read();
    }
  }
  */
  
  Serial.println("[GPS] ========== Konfiguration abgeschlossen (gespeichert in RAM+BBR+Flash) ==========");
}

void UbxM10::resetFrame_() {
  msgClass_ = 0;
  msgId_ = 0;
  payloadLen_ = 0;
  payloadPos_ = 0;
  ckA_ = 0;
  ckB_ = 0;
}

void UbxM10::updateChecksum_(uint8_t byte) {
  ckA_ = static_cast<uint8_t>(ckA_ + byte);
  ckB_ = static_cast<uint8_t>(ckB_ + ckA_);
}

void UbxM10::handleFrame_() {
  // NAV-PVT
  if (msgClass_ == 0x01 && msgId_ == 0x07 && payloadLen_ >= 92) {
    // Offsets per UBX-NAV-PVT.
    year_ = u16le_(payload_ + 4);
    month_ = payload_[6];
    day_ = payload_[7];
    hour_ = payload_[8];
    minute_ = payload_[9];
    second_ = payload_[10];
    // valid flag at 11, tAcc at 12
    nano_ = i32le_(payload_ + 16); // Added nano parsing

    fixType_ = payload_[20];
    const uint8_t flags = payload_[21];
    gnssFixOk_ = (flags & 0x01) != 0;
    numSV_ = payload_[23];

    lon1e7_ = i32le_(payload_ + 24);
    lat1e7_ = i32le_(payload_ + 28);

    hMSLmm_ = i32le_(payload_ + 36);
    gSpeedMmS_ = u32le_(payload_ + 60);
    headMot1e5_ = i32le_(payload_ + 64);

    newFix_ = true;
    return;
  }

  // NAV-DOP
  if (msgClass_ == 0x01 && msgId_ == 0x04 && payloadLen_ >= 18) {
    // vDOP at offset 10, 0.01
    vDOP_1e2_ = u16le_(payload_ + 10);
    // hDOP at offset 12, 0.01
    hDOP_1e2_ = u16le_(payload_ + 12);
    return;
  }
}

uint16_t UbxM10::u16le_(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t UbxM10::u32le_(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

int32_t UbxM10::i32le_(const uint8_t *p) {
  return static_cast<int32_t>(u32le_(p));
}

uint8_t UbxM10::nmeaChecksum_(const String &body) {
  uint8_t cs = 0;
  for (size_t i = 0; i < body.length(); i++) {
    cs ^= static_cast<uint8_t>(body.charAt(i));
  }
  return cs;
}

String UbxM10::formatLatLon_(int32_t deg1e7, bool isLat, char &hemi) {
  const bool neg = deg1e7 < 0;
  const double deg = fabs(deg1e7 / 1e7);

  const int degInt = static_cast<int>(deg);
  const double minutes = (deg - degInt) * 60.0;

  if (isLat) {
    hemi = neg ? 'S' : 'N';
    char buf[16];
    // ddmm.mmmm
    snprintf(buf, sizeof(buf), "%02d%07.4f", degInt, minutes);
    return String(buf);
  }

  hemi = neg ? 'W' : 'E';
  char buf[16];
  // dddmm.mmmm
  snprintf(buf, sizeof(buf), "%03d%07.4f", degInt, minutes);
  return String(buf);
}

String UbxM10::formatTime_(uint8_t hh, uint8_t mm, uint8_t ss) {
  char buf[12];
  // hhmmss.ss (we keep .00)
  snprintf(buf, sizeof(buf), "%02u%02u%02u.00", hh, mm, ss);
  return String(buf);
}

String UbxM10::withChecksum_(const String &body) {
  const uint8_t cs = nmeaChecksum_(body);
  char csBuf[5];
  snprintf(csBuf, sizeof(csBuf), "*%02X", cs);
  return String("$") + body + String(csBuf);
}

void UbxM10::sendUbx_(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  auto upd = [&](uint8_t b) {
    ckA = static_cast<uint8_t>(ckA + b);
    ckB = static_cast<uint8_t>(ckB + ckA);
  };

  serial_.write(0xB5);
  serial_.write(0x62);

  serial_.write(cls);
  serial_.write(id);
  upd(cls);
  upd(id);

  uint8_t len1 = static_cast<uint8_t>(len & 0xFF);
  uint8_t len2 = static_cast<uint8_t>((len >> 8) & 0xFF);
  serial_.write(len1);
  serial_.write(len2);
  upd(len1);
  upd(len2);

  for (uint16_t i = 0; i < len; i++) {
    serial_.write(payload[i]);
    upd(payload[i]);
  }

  serial_.write(ckA);
  serial_.write(ckB);
}
