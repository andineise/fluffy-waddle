#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <ESP32-TWAI-CAN.hpp>
#include "ble_nmea_uart.h"
#include "ubx_m10.h"

// Pinout
#define DIMM     36  
#define SENSOR   37  
#define CAN_RX    4  
#define CAN_TX    5  
#define GPS_RX   32  
#define GPS_TX   33  
#define TFT_BL   15  
#define GPS_SERIAL Serial1 

// Globals
static const bool RACECHRONO_PRO_MODE = true; // true = RaceChrono DIY Protocol (GPS + CAN, Pro required)
static constexpr bool ENABLE_GPS_BLE = true; // Set false to disable GPS+BLE together

String BtName;
BleNmeaUart BleNmea;
UbxM10 GpsUbx(GPS_SERIAL);
bool canInitialized = false;  
volatile uint32_t lastValidFixMs = 0;

static constexpr bool CAN_DEBUG_IDS = false;
static constexpr uint32_t CAN_DEBUG_INTERVAL_MS = 250;

// ==== PERFORMANCE TUNING ====
// Task Priorities (higher = more important)
static constexpr UBaseType_t PRIO_GPS = 4;   // Höchste - GPS→BLE ist zeitkritisch (20Hz)
static constexpr UBaseType_t PRIO_GUI = 2;   // Mittel - Display Update (50Hz reicht)
static constexpr UBaseType_t PRIO_CAN = 1;   // Niedrig - CAN ist nur 10Hz

// Task Timing (ms)
static constexpr uint32_t GPS_POLL_INTERVAL_MS = 2;   // 500Hz polling für 20Hz GPS
static constexpr uint32_t GUI_UPDATE_INTERVAL_MS = 20; // 50Hz Display (menschliche Wahrnehmung)
static constexpr uint32_t CAN_POLL_INTERVAL_MS = 5;    // 200Hz polling für 10Hz CAN

// Stack Sizes
static constexpr uint32_t STACK_GPS = 6144;  // GPS + BLE braucht mehr Stack
static constexpr uint32_t STACK_GUI = 8192;  // LVGL braucht viel Stack
static constexpr uint32_t STACK_CAN = 3072;  // CAN braucht weniger

// ==== CAN DATA SOURCE ====
// CAN_SOURCE_AUTO: Auto-detect ECU type from received CAN IDs
// CAN_SOURCE_ME:   Motorsport Electronics ECU (IDs 768-775)
// CAN_SOURCE_MS:   MegaSquirt ECU (IDs 1512-1516)
// CAN_SOURCE_LINK: Link/G4+ ECU (IDs 1536-1537)
enum CanSource { CAN_SOURCE_AUTO = 0, CAN_SOURCE_ME, CAN_SOURCE_MS, CAN_SOURCE_LINK };
static constexpr CanSource CAN_DATA_SOURCE    = CAN_SOURCE_AUTO;
static constexpr uint32_t  CAN_SOURCE_TIMEOUT_MS = 5000; // ms ohne Daten bevor Quelle wechselt
static volatile CanSource  activeCanSource    = CAN_SOURCE_LINK;

int d = 0; 
int sensorPin = 37;  
int dimPin = 36;  
int dimState = 0;
int brightness = 0;

// Mutex for LVGL thread safety
SemaphoreHandle_t lvglMutex;

// Cached formatted strings to avoid String allocation in GUI task
static char strClt[8], strOilT[8], strBoost[8], strOilP[8], strAfr[8], strEgt[8], strIat[8];
static volatile bool canDataUpdated = false;

// Display
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); 

// Data Structs
struct MeData
{
    //ID 768
    int rpm, tps, map, iat; 
    //ID 769
    int rpm_hardlimit, LambdaTrim, FuelEthPerc; 
    float AFRCurrent1, AFRCurrent2, AFRTarget;  
    //ID 770
    int PriInjPw, PriInjAngle, IgnDwell, IgnAdvAngle;
    //ID 771
    int PriInjDuty, SecInjAngle, SecInjDuty, SecInjPw, BoostCtrlDuty;
    //ID 772
    int OilT, OilP, Clt;
    float VBat;
    //ID 773
    int GearPos, VehicleSpeed, EpsEvMask;
    float MapTarget;
    //ID 774
    int KnockReading, KnockIgnAdvMod, FuelPress, FuelTemp, EGT1;
    //ID 775
    int EGT2, GPT1, GPT2;
    //ID 832
    int VehicleSpeed2;
};

struct MsData
{
    //ID 1512 0x5E8
    int rpm, clt, tps, iat;
    float MAP;
    //ID1513 0x5E9
    int pw1, pw2, mat, adv_deg;
    //ID1514 0x5EA
    int EGOcor1, EGT1, pwseq1;
    float afrtgt1, afr;
    //ID1515 0x5EB
    float batt, knk_rtd;
    int sensors1, sensors2;
    //ID1516 0x5EC
    int VSS1, tc_retard, launch_timing;
};

struct LinkData
{
  //ID1536
  int clt;
  int iat;
  int oilP;
  int oilT;
  //ID1537
  int map;
  int lambda;
  int egt;
};

MeData Me;
MsData MS;
LinkData Link;

// Prototypes
bool processCANPacket();  // Returns true if display-relevant data was updated
static void setGpsUiValid(bool valid);

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;
    bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

    if( !touched ) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

// -------------------------------------------------------------------------
// Tasks
// -------------------------------------------------------------------------

void TaskGUI(void *pvParameters) {
    (void) pvParameters;
    
    static uint32_t lastBrightnessUpdate = 0;
    static bool lastGpsValid = false;
    
    // Einmalig Brightness setzen
    analogWrite(TFT_BL, 200);

    for (;;) {
        if (xSemaphoreTake(lvglMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            
            // GPS Status UI Update (nur bei Änderung)
            const bool gpsValid = (lastValidFixMs != 0 && (millis() - lastValidFixMs < 10000));
            if (gpsValid != lastGpsValid) {
                setGpsUiValid(gpsValid);
                lastGpsValid = gpsValid;
            }

            // Update UI Labels nur wenn CAN-Daten aktualisiert wurden
            if (canDataUpdated) {
                const CanSource src = (CAN_DATA_SOURCE == CAN_SOURCE_AUTO)
                                      ? activeCanSource : CAN_DATA_SOURCE;

                switch (src) {
                    case CAN_SOURCE_ME:
                        snprintf(strClt,   sizeof(strClt),   "%d",   Me.Clt);
                        snprintf(strOilT,  sizeof(strOilT),  "%d",   Me.OilT);
                        snprintf(strBoost, sizeof(strBoost), "%d",   Me.map);
                        snprintf(strOilP,  sizeof(strOilP),  "%d",   Me.OilP);
                        snprintf(strAfr,   sizeof(strAfr),   "%.2f", Me.AFRCurrent1);
                        snprintf(strEgt,   sizeof(strEgt),   "%d",   Me.EGT1);
                        snprintf(strIat,   sizeof(strIat),   "%d",   Me.iat);
                        break;
                    case CAN_SOURCE_MS:
                        snprintf(strClt,   sizeof(strClt),   "%d",   MS.clt);
                        snprintf(strOilT,  sizeof(strOilT),  "%d",   MS.sensors1);     
                        snprintf(strBoost, sizeof(strBoost), "%.1f", MS.MAP);
                        snprintf(strOilP,  sizeof(strOilP),  "%d",   MS.sensors2);
                        snprintf(strAfr,   sizeof(strAfr),   "%.2f", MS.afr);
                        snprintf(strEgt,   sizeof(strEgt),   "%d",   MS.EGT1);
                        snprintf(strIat,   sizeof(strIat),   "%d",   MS.mat);
                        break;
                    default: // CAN_SOURCE_LINK
                        snprintf(strClt,   sizeof(strClt),   "%d",   Link.clt);
                        snprintf(strOilT,  sizeof(strOilT),  "%d",   Link.oilT);
                        snprintf(strBoost, sizeof(strBoost), "%d",   Link.map);
                        snprintf(strOilP,  sizeof(strOilP),  "%.1f", Link.oilP * 0.01f);
                        snprintf(strAfr,   sizeof(strAfr),   "%.2f", Link.lambda * 0.001f);
                        snprintf(strEgt,   sizeof(strEgt),   "%d",   Link.egt);
                        snprintf(strIat,   sizeof(strIat),   "%d",   Link.iat);
                        break;
                }

                lv_label_set_text(ui_coolant,     strClt);
                lv_label_set_text(ui_oiltemp,     strOilT);
                lv_label_set_text(ui_Boost,       strBoost);
                lv_label_set_text(ui_OilPressure, strOilP);
                lv_label_set_text(ui_AFR,         strAfr);
                lv_label_set_text(ui_EGT,         strEgt);
                lv_label_set_text(ui_IAT,         strIat);

                canDataUpdated = false;
            }
            
            // Maintain LVGL
            lv_timer_handler();
            
            xSemaphoreGive(lvglMutex);
        }
        
        // 50Hz ist ausreichend für flüssige Anzeige
        vTaskDelay(pdMS_TO_TICKS(GUI_UPDATE_INTERVAL_MS));
    }
}

void TaskCAN(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        if (canInitialized) {
            // Verarbeite alle verfügbaren CAN Frames
            if (processCANPacket()) {
                canDataUpdated = true;  // Signal an GUI Task
            }
        }
        // 200Hz polling reicht für 10Hz CAN-Daten locker aus
        vTaskDelay(pdMS_TO_TICKS(CAN_POLL_INTERVAL_MS));
    }
}

void TaskGPS(void *pvParameters) {
    (void) pvParameters;
    
    // Vorallokierte NMEA Strings für bessere Performance
    String nmeaGga, nmeaRmc;
    nmeaGga.reserve(96);  // GGA ist max ~82 Zeichen
    nmeaRmc.reserve(80);  // RMC ist max ~70 Zeichen

    static uint32_t lastGpsDebug = 0;
    static uint32_t gpsPacketCount = 0;
    static uint32_t gpsSerialBytes = 0;
    static uint32_t lastBleSend = 0;
    
    for (;;) {
        // Poll GPS Serial - poll() gibt true zurück wenn NAV-PVT empfangen
        
        // Check wie viele Bytes im Serial Buffer
        int available = GPS_SERIAL.available();
        if (available > 0) {
            gpsSerialBytes += available;
        }
        
        // Prozessiere alle verfügbaren Bytes und sende sofort bei neuem UBX-Paket
        while (GpsUbx.poll()) {
            if (GpsUbx.consumeNewFix()) {
                gpsPacketCount++;
                
                // Sofort nach jedem UBX NAV-PVT Paket an RaceChrono senden!
#if ENABLE_GPS_BLE
                if (BleNmea.isConnected()) {
                    lastBleSend = millis();
                    
                    const bool gpsOk = GpsUbx.hasFix() && (GpsUbx.numSats() > 2);
                    if (gpsOk) {
                        lastValidFixMs = millis();
                    }

                    // BLE Übertragung - direkt nach UBX Empfang = minimale Latenz
                    if (RACECHRONO_PRO_MODE) {
                        BleNmea.updateGps(GpsUbx);
                    } else {
                        BleNmea.sendLocationSpeed(GpsUbx);
                    }
                }
#else
                (void) lastBleSend;
                const bool gpsOk = GpsUbx.hasFix() && (GpsUbx.numSats() > 2);
                if (gpsOk) {
                    lastValidFixMs = millis();
                }
#endif
            }
        }
        
        // Debug: GPS-Paket-Rate alle 30 Sekunden anzeigen
        if (millis() - lastGpsDebug >= 30000) {
            Serial.printf("[GPS] %.1f Hz (%lu packets/30s)\n", 
                          gpsPacketCount / 30.0f, gpsPacketCount);
            gpsPacketCount = 0;
            gpsSerialBytes = 0;
            lastGpsDebug = millis();
        }
        
        // 2ms Delay = 500Hz Loop Rate, fängt 20Hz GPS sicher ab
        // Kurzes Delay ist kritisch für minimale GPS→BLE Latenz!
        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_INTERVAL_MS));
    }
}

// -------------------------------------------------------------------------
// Setup & Loop
// -------------------------------------------------------------------------

void setup() {
    uint32_t id = 0;
    for(int i=0; i<17; i=i+8) {
        id |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    BtName = "NiceTrack "+String(id);
    
    Serial.begin(921600);  // Schnellere Baudrate für Debug
    Serial.println("\n\n=== CanGauge Starting ===");
    
    // Create Mutex for shared resources (LVGL)
    lvglMutex = xSemaphoreCreateMutex();

    // GPS + BLE Init
    if constexpr (ENABLE_GPS_BLE) {
        GpsUbx.begin(38400, GPS_RX, GPS_TX);
        GpsUbx.configureUbx();
        BleNmea.begin(BtName, RACECHRONO_PRO_MODE);
    } else {
        Serial.println("GPS + BLE disabled by configuration");
    }

    pinMode(TFT_BL, OUTPUT);
    pinMode(dimPin, INPUT);
    
    Serial.println("Initializing CAN...");
    
    // CAN Init
    ESP32Can.setPins(CAN_TX, CAN_RX);
    ESP32Can.setSpeed(TWAI_SPEED_500KBPS);
    ESP32Can.setRxQueueSize(50);
    
    if (!ESP32Can.begin()) {
        Serial.println("CAN init failed - Continuing without CAN...");
        canInitialized = false;
    } else {
        Serial.println("CAN initialized successfully");
        canInitialized = true;
    }
    
    Serial.print("BluetoothName: ");
    Serial.println(BtName);

    // LVGL & TFT Init
    lv_init();
    tft.begin();
    tft.setRotation( 2 ); 

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );
    
    ui_init();

    Serial.println( "Setup done, starting tasks..." );

    // ===== OPTIMIERTE TASK-STRUKTUR =====
    // Core 0: GPS + BLE (Zeitkritisch! NimBLE läuft auch auf Core 0)
    // Core 1: GUI + CAN (Weniger zeitkritisch)
    //
    // Prioritäten:
    //   GPS (4) > GUI (2) > CAN (1) > Idle (0)
    //
    // GPS→BLE ist die kritischste Verbindung (20Hz, minimale Latenz)
    // CAN ist nur 10Hz, GUI nur 50Hz nötig
    
    // GPS Task - HÖCHSTE Priorität, Core 0 (wo BLE läuft)
    if constexpr (ENABLE_GPS_BLE) {
        xTaskCreatePinnedToCore(TaskGPS, "GPS", STACK_GPS, NULL, PRIO_GPS, NULL, 0);
    } else {
        Serial.println("Skipping GPS task because GPS + BLE are disabled");
    }
    
    // GUI Task - Mittlere Priorität, Core 1
    xTaskCreatePinnedToCore(TaskGUI, "GUI", STACK_GUI, NULL, PRIO_GUI, NULL, 1);
    
    // CAN Task - Niedrigste Priorität, Core 1
    xTaskCreatePinnedToCore(TaskCAN, "CAN", STACK_CAN, NULL, PRIO_CAN, NULL, 1); 
}

void loop() {
    vTaskDelete(NULL); // Loop task not needed anymore
}

// -------------------------------------------------------------------------
// Helpers
// -------------------------------------------------------------------------

static void setGpsUiValid(bool valid) {
    // This MUST be called within lvglMutex lock as it touches UI
    lv_obj_set_style_text_color(ui_GPS, valid ? lv_color_hex(0xFFFFFF)
                                             : lv_color_hex(0x000000),
                                LV_PART_MAIN);
}

bool processCANPacket() {
  static uint32_t lastDebugMs = 0;
  static uint32_t framesSinceDebug = 0;
  static uint32_t lastMeMs = 0, lastMsMs = 0, lastLinkMs = 0;
  bool displayUpdated = false;  // Track if display-relevant data changed

  CanFrame rxFrame;
  while (ESP32Can.readFrame(rxFrame, 0)) {
    const int id = rxFrame.identifier;
    const uint8_t* CanB = rxFrame.data;
    framesSinceDebug++;

    if (CAN_DEBUG_IDS) {
      const uint32_t nowMs = millis();
      if (nowMs - lastDebugMs >= CAN_DEBUG_INTERVAL_MS) {
        Serial.print("CAN rx/s ~");
        Serial.print(framesSinceDebug * (1000 / CAN_DEBUG_INTERVAL_MS));
        Serial.print("  queued=");
        Serial.print(ESP32Can.inRxQueue());
        Serial.print("  last=0x");
        Serial.println(id, HEX);
        framesSinceDebug = 0;
        lastDebugMs = nowMs;
      }
    }

    switch (id)
    {
    case 768:
      Me.rpm = ((CanB[1] << 8) | CanB[0]);
      Me.tps = ((CanB[3] << 8) | CanB[2])*0.1;
      Me.map = int(((CanB[5] << 8) | CanB[4])*0.01);
      Me.iat = ((CanB[7] << 8)| CanB[6])*0.1;
    break;

   case 769:
      Me.AFRCurrent1   = float(((CanB[2] * 0.05) + 7.5)/14.7);
      Me.AFRTarget     = float(((CanB[6] * 0.05) + 7.5)/14.7);
    break;

    case 770:
      Me.IgnAdvAngle = ((CanB[1] << 8) | CanB[0])*0.1;
    break;

    case 772:
      Me.OilT = float((CanB[1] << 8) | CanB[0]) * 0.1;
      Me.OilP = float((CanB[3] << 8) | CanB[2]) * 0.1;
      Me.Clt  = float((CanB[5] << 8) | CanB[4]) * 0.1;
      Me.VBat = float((CanB[7] << 8) | CanB[6]) * 0.1;
      lastMeMs = millis(); displayUpdated = true;
    break;

    case 773:
      Me.VehicleSpeed = ((CanB[2] << 16) | (CanB[1] << 8) | CanB[0]) * 0.008;
    break;

    case 774:
      Me.EGT1 = (float)((CanB[1] << 8) | CanB[0]) * 0.1;
    break;

    case 1512:
      MS.MAP = float((((CanB[0] << 8) | CanB[1])*0.001)-1.0);
      MS.rpm = ((CanB[2] << 8) | CanB[3]);
      MS.clt = ((((CanB[4] << 8) | CanB[5])*0.1)-32)*5/9;
      MS.tps = ((CanB[6] << 8) | CanB[7])*0.1;
      lastMsMs = millis(); displayUpdated = true;
      break;

    case 1513:
      MS.pw1 = ((CanB[0] << 8)| CanB[1])*0.001;
      MS.pw2 = ((CanB[2] << 8)| CanB[3])*0.001;
      MS.mat = ((((CanB[4] << 8)| CanB[5])*0.1)-32)*5/9;
      MS.adv_deg =((CanB[6] << 8)| CanB[7])*0.1;
      break;

  case 1514:
      MS.afrtgt1 = float((CanB[0]*0.1)/14.7);
      MS.afr     = float((CanB[1]*0.1)/14.7);
      MS.EGOcor1 = ((CanB[2] << 8)| + CanB[3])*0.1;
      MS.EGT1    = ((((CanB[4] << 8)| + CanB[5])*0.1)-32)*5/9;
      MS.pwseq1  = ((CanB[6] << 8)| CanB[7])*0.001;
      break;

  case 1515:
      MS.batt      = float((CanB[0] << 8)| CanB[1])*0.1;
      MS.sensors1  = ((CanB[2] << 8)| CanB[3])*0.1;
      MS.sensors2  = ((CanB[4] << 8)| CanB[5])*0.1;
      MS.knk_rtd   =  CanB[6]/10;
  break;

  case 1516:
      MS.VSS1 = (((CanB[0] << 8)| CanB[1])*0.1)*3.6;
      MS.tc_retard     = ((CanB[2] << 8)| CanB[3])*0.1;
      MS.launch_timing = ((CanB[4] << 8)| CanB[5])*0.1;
      break;

    case 1536: // Link Monsoon Stream 1
      Link.clt  = (int16_t)((CanB[0] << 8) | CanB[1]);
      Link.iat  = (int16_t)((CanB[2] << 8) | CanB[3]);
      Link.oilP = (CanB[4] << 8) | CanB[5];
      Link.oilT = (int16_t)((CanB[6] << 8) | CanB[7]);
      lastLinkMs = millis(); displayUpdated = true;
      if (RACECHRONO_PRO_MODE) {
          BleNmea.sendCanData(1536, CanB, 8);
      }
      break;

    case 1537: // Link Monsoon Stream 2
      Link.map    = (int16_t)((CanB[0] << 8) | CanB[1]);
      Link.lambda = (CanB[2] << 8) | CanB[3];
      Link.egt    = (CanB[4] << 8) | CanB[5];
      lastLinkMs = millis(); displayUpdated = true;
      if (RACECHRONO_PRO_MODE) {
          BleNmea.sendCanData(1537, CanB, 8);
      }
      break;

    default:
      break;
    }
  }

  // Auto-detect active ECU source from most recently seen CAN IDs
  if (CAN_DATA_SOURCE == CAN_SOURCE_AUTO) {
    const uint32_t now = millis();
    const bool meActive  = lastMeMs   && (now - lastMeMs)   < CAN_SOURCE_TIMEOUT_MS;
    const bool msActive  = lastMsMs   && (now - lastMsMs)   < CAN_SOURCE_TIMEOUT_MS;
    const bool lnkActive = lastLinkMs && (now - lastLinkMs) < CAN_SOURCE_TIMEOUT_MS;
    if      (lnkActive) activeCanSource = CAN_SOURCE_LINK;
    else if (msActive)  activeCanSource = CAN_SOURCE_MS;
    else if (meActive)  activeCanSource = CAN_SOURCE_ME;
  }

  return displayUpdated;
}
