#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Firebase_ESP_Client.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <vector>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"

#include "addons/RTDBHelper.h"
#include "addons/TokenHelper.h"

// ================= WIFI & FIREBASE =================
// #define WIFI_SSID "CICI_BIZNET"
// #define WIFI_PASSWORD "cici2024"

#define WIFI_SSID "iPhone"
#define WIFI_PASSWORD "kepobanget"

// #define WIFI_SSID "iPhone 17"
// #define WIFI_PASSWORD "00000000"


#define DATABASE_URL "https://sistem-monitoring-73612-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define DATABASE_SECRET "H8XHQOUjm9H9Ko8RKn0SmQPlOB1CFueofoIPoUGI"

const char *APPS_SCRIPT_URL =
    "https://script.google.com/macros/s/AKfycbx6RoKKThAadbcgRUzM5IipWgVHVa2FG0vaWauENG5QggE-W83rdoKGJpUZLE6BGT-4vQ/exec";

const char *FIREBASE_REALTIME_PATH = "/sistem_air/realtime";
const char *FIREBASE_HISTORY_PATH = "/sistem_air/history";
const char *FIREBASE_COMMAND_PATH = "/sistem_air/commands/kuras_tandon";
const char *FIREBASE_COMMANDS_PATH = "/sistem_air/commands";
const char *FIREBASE_PUMP_AWAL_PATH = "/sistem_air/commands/pump_awal_12v";

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= PIN SENSOR =================
// Penampungan awal: air pertama masuk, dipantau turbidity dan HC-SR04.
#define TURB_AWAL_PIN        34
#define HC_AWAL_TRIG_PIN     23
#define HC_AWAL_ECHO_PIN     25

// Tandon akhir: air hasil routing, dipantau pH, turbidity, suhu, dan JSN-SR04T.
#define TURB_TANDON_PIN      35
#define PH_TANDON_PIN        32
#define SUHU_TANDON_PIN      4
#define JSN_TANDON_TRIG_PIN  18
#define JSN_TANDON_ECHO_PIN  19

// ================= PIN AKTUATOR =================
// Modul relay 4-channel yang dipakai aktif LOW:
// LOW  = relay aktif, HIGH = relay mati.
// Polaritas ini harus benar agar perintah OFF saat boot tidak justru
// menyalakan semua relay.
#define RELAY_PUMP_AWAL_12V_PIN      26 //IN 1
#define RELAY_PUMP_KURAS_5V_PIN      27 //IN 2
#define RELAY_SOLENOID_BERSIH_PIN    14 //IN 3
#define RELAY_SOLENOID_KERUH_PIN     33 //IN 4

#define RELAY_ON   LOW
#define RELAY_OFF  HIGH

// Mode ini dipakai untuk setting sensor/kalibrasi:
// semua relay dipaksa OFF, pembacaan sensor dan Firebase tetap berjalan.
const bool MODE_KALIBRASI_SENSOR = false;
const bool FIREBASE_HISTORY_AKTIF = !MODE_KALIBRASI_SENSOR;
const bool SIMPAN_ANTREAN_FIREBASE = !MODE_KALIBRASI_SENSOR;

// ================= DS18B20 =================
OneWire oneWire(SUHU_TANDON_PIN);
DallasTemperature sensors(&oneWire);

// ================= NILAI SENSOR =================
// Penampungan awal
float kekeruhan_awal = 0;
float kekeruhan_awal_raw = 0;
float jarak_air_awal = 0;
float jarak_air_awal_raw = 0;
float adc_turb_awal = 0;
float volt_turb_awal = 0;
float volt_turb_awal_raw = 0;

// Tandon akhir
float kekeruhan_tandon = 0;
float kekeruhan_tandon_raw = 0;
float ph_tandon = 0;
float suhu_tandon = 0;
float jarak_air_tandon = 0;
float adc_turb_tandon = 0;
float volt_turb_tandon = 0;
float volt_turb_tandon_raw = 0;
float adc_ph_tandon = 0;
float volt_ph_tandon = 0;

// ================= KALIBRASI SENSOR =================
// pH air bersih sebelumnya terbaca 6.50; offset dinaikkan +0.75
// supaya air bersih berada di sekitar 7.25.
float calibration_value = 22.35;

// Penampungan awal kosong: jarak sensor ke dasar/permukaan kosong = 30 cm.
// Nilai level awal yang ditampilkan adalah tinggi air: 30 cm - jarak sensor.
const float TINGGI_KOSONG_PENAMPUNGAN_AWAL_CM = 30.0;
// Level awal kosong terbaca 28-29 cm, kondisi aktual diset menjadi 30 cm.
const float KALIBRASI_LEVEL_AWAL_OFFSET_CM = 1;

// Turbidity awal: baseline air bersih dikalibrasi ulang.
const int TURB_AWAL_ADC_BITS = 12;
const float TURB_AWAL_ADC_MAX = 4095.0;
const float TURB_AWAL_VREF = 3.3;
const uint8_t TURB_AWAL_AVG_SAMPLES = 32;
const uint8_t TURB_TANDON_AVG_SAMPLES = 64;
// Median dari tiga hasil trimmed-average membuang satu lonjakan sesaat,
// tetapi perubahan air yang nyata tetap mulai diikuti pada pembacaan kedua.
const uint8_t TURB_TANDON_TEMPORAL_SAMPLES = 3;
const float TURB_AWAL_DEFAULT_ADC_CLEAR = 4000.0;
const float TURB_TANDON_DEFAULT_ADC_CLEAR = 4000.0;
const float TURB_AWAL_NTU_MAX = 500.0;
const float TURB_AWAL_NTU_THRESHOLD = 25.0;
// Deadband awal dibuat sangat kecil agar perubahan kekeruhan ringan tetap
// terbaca. Tandon memakai deadband moderat karena sensor akhirnya lebih rentan
// noise dari suplai dan aktuator.
const float TURB_AWAL_ZERO_NTU_BAND = 0.5;
const float TURB_TANDON_ZERO_NTU_BAND = 10.0;
const float TURB_AWAL_NTU_GAIN = 12.0;
const float TURB_TANDON_NTU_GAIN = 1.0;
// Respons dibuat asimetris (fast attack, slow release): cepat saat air
// bertambah keruh (ADC turun), tetapi nilai NTU turun perlahan saat air
// kembali jernih. Alpha release awal 0,0029 memberi waktu paruh sekitar
// 4 menit pada pembacaan normal 1 detik, atau sekitar 1 menit di mode raw
// 250 ms, sehingga nilai tidak langsung jatuh ke 0 NTU.
const float TURB_AWAL_EMA_ALPHA_TURBID = 0.95;
const float TURB_AWAL_EMA_ALPHA_CLEAR = 0.0012;
const float TURB_TANDON_EMA_ALPHA_TURBID = 0.30;
const float TURB_TANDON_EMA_ALPHA_CLEAR = 0.0115;
// Sensor turbidity menghasilkan ADC lebih rendah saat air makin keruh.
// Kenaikan ADC di atas baseline air bersih bukan kekeruhan dan harus tetap 0 NTU.
const bool TURB_AWAL_BIDIRECTIONAL_ADC = false;
const bool TURB_TANDON_BIDIRECTIONAL_ADC = false;
const float TURB_AWAL_MAX_ADC_STEP = 250.0;
const float TURB_TANDON_MAX_ADC_STEP = 0.0;
const uint32_t TURB_AWAL_READ_MS = 250;

const int TURB_AWAL_EEPROM_SIZE = 16;
const uint8_t TURB_AWAL_EEPROM_MAGIC = 0xCE;
const int TURB_AWAL_ADDR_MAGIC = 0;
const int TURB_AWAL_ADDR_ADC_CLEAR = 1;
const int TURB_TANDON_ADDR_ADC_CLEAR = 5;

float turbAwalAdcClear = TURB_AWAL_DEFAULT_ADC_CLEAR;
float turbTandonAdcClear = TURB_TANDON_DEFAULT_ADC_CLEAR;
float turbAwalEmaAdc = 0.0;
float turbTandonEmaAdc = 0.0;
bool turbAwalEmaReady = false;
bool turbTandonEmaReady = false;
float turbTandonTemporalAdc[TURB_TANDON_TEMPORAL_SAMPLES] = {0.0f, 0.0f, 0.0f};
uint8_t turbTandonTemporalIndex = 0;
bool turbTandonTemporalReady = false;
bool turbAwalShowRaw = false;
uint32_t turbAwalReadingSeq = 0;

// ================= PARAMETER SISTEM =================
// Ultrasonik awal membaca jarak sensor ke permukaan air, lalu dikonversi
// menjadi tinggi air dari dasar penampungan.
// Pump awal menyala saat tinggi air awal >=20 cm, lalu routing air:
// NTU <=25 ke solenoid bersih, NTU >25 ke solenoid keruh.
// Aliran berhenti saat tinggi air <=1 cm.
const float BATAS_AIR_AWAL_MULAI_CM = 20.0;
const float BATAS_AIR_AWAL_STOP_CM = 1.0;
const float BATAS_TANDON_BOLEH_KURAS_CM = 35.0;
const float BATAS_TANDON_STOP_KURAS_CM = 45.0;

const float JARAK_MIN_VALID_CM = 2.0;
const float JARAK_MAX_VALID_CM = 450.0;
// Filter dasar JSN-SR04T tandon. Rentang valid umum tetap dipakai sementara
// karena tahap ini hanya menstabilkan noise, belum mengalibrasi posisi sensor.
const uint8_t JSN_TANDON_SAMPLE_COUNT = 7;
const uint8_t JSN_TANDON_MIN_VALID_SAMPLES = 3;
const uint16_t JSN_TANDON_SAMPLE_GAP_MS = 60;
const float JSN_TANDON_FILTER_ALPHA = 0.20;
const float JSN_TANDON_MAX_SINGLE_JUMP_CM = 6.0;
const float JSN_TANDON_JUMP_TOLERANCE_CM = 3.0;
const uint8_t JSN_TANDON_JUMP_CONFIRM_READS = 2;
const uint8_t JSN_TANDON_MAX_FAILED_READS = 4;
const uint32_t SENSOR_NON_LEVEL_SETTLE_MS = 3000;
const uint8_t SENSOR_NON_LEVEL_SMOOTH_READS = 5;
const float SENSOR_NON_LEVEL_RESUME_ALPHA = 0.12;
const float SUHU_MIN_VALID_C = -10.0;
const float SUHU_MAX_VALID_C = 85.0;
const float PH_MIN_VALID = 0.0;
const float PH_MAX_VALID = 14.0;
const float FILTER_SENSOR_ALPHA = 0.35;

// Routing awal: air >25 NTU dialirkan lewat solenoid keruh.
// Setelah masuk jalur keruh, kembali ke bersih saat NTU <=20 supaya
// relay tidak cepat bolak-balik ketika pembacaan berada dekat batas.
const float BATAS_KERUH_AWAL = 25.0;
const float BATAS_KERUH_AWAL_KEMBALI_BERSIH = 20.0;
const uint8_t TURB_AWAL_KERUH_CONFIRM_READS = 2;
const uint8_t TURB_AWAL_BERSIH_CONFIRM_READS = 8;
const uint32_t TURB_AWAL_PUMP_STARTUP_IGNORE_MS = 3000;
const uint32_t SOLENOID_SWITCH_DEADTIME_MS = 400;
const uint32_t SOLENOID_KERUH_AKTIF_MS = 5000;
const uint32_t SOLENOID_KERUH_MATI_MS = 10000;

// ================= STATUS SISTEM =================
bool pumpAwal12vAktif = false;
bool pumpKuras5vAktif = false;
bool solenoidKeruhAktif = false;
bool solenoidBersihAktif = false;
bool jalurAwalKeruhLatch = false;
bool solenoidKeruhPulseAktif = true;
uint32_t solenoidKeruhPulseLastMs = 0;
uint32_t pumpAwalLastOnMs = 0;
uint32_t turbAwalDecisionSeq = 0;
uint8_t turbAwalKeruhConfirmCount = 0;
uint8_t turbAwalBersihConfirmCount = 0;
bool jalurAwalKeputusanSiap = false;

enum SolenoidRoute {
  SOLENOID_ROUTE_NONE,
  SOLENOID_ROUTE_CLEAN,
  SOLENOID_ROUTE_TURBID
};

enum SolenoidTransitionState {
  SOLENOID_TRANSITION_IDLE,
  SOLENOID_TRANSITION_DEADTIME
};

SolenoidRoute requestedSolenoidRoute = SOLENOID_ROUTE_NONE;
SolenoidTransitionState solenoidTransitionState = SOLENOID_TRANSITION_IDLE;
uint32_t solenoidTransitionStartedMs = 0;

bool commandModeManual = false;
bool commandModeManualValid = true;
bool commandPumpAwal12v = false;
bool commandPumpAwal12vValid = false;
SolenoidRoute commandSolenoidRoute = SOLENOID_ROUTE_NONE;
bool commandSolenoidRouteValid = false;
bool appliedModeManual = false;
bool controlModeInitialized = false;
bool firebaseControlConnected = false;
bool firebaseInitialPumpResetPending = true;

bool actuatorStatusDirty = true;
uint32_t actuatorStatusGeneration = 0;

bool triggerKurasTandon = false;
bool perintahKurasTandon = false;
bool relayBaruOff = false;

// ================= TIMER =================
unsigned long lastLcdUpdate = 0;
unsigned long lastSerialUpdate = 0;
byte lcdPage = 0;

const unsigned long LCD_INTERVAL = 1000;
const unsigned long SERIAL_INTERVAL = 1000;
uint32_t turbidityRawLastMs = 0;

const TickType_t SENSOR_TASK_INTERVAL = pdMS_TO_TICKS(1000);
const TickType_t CONTROL_TASK_INTERVAL = pdMS_TO_TICKS(100);
const TickType_t LCD_TASK_INTERVAL = pdMS_TO_TICKS(200);
const TickType_t SERIAL_COMMAND_TASK_INTERVAL = pdMS_TO_TICKS(50);
const TickType_t SERIAL_LOG_TASK_INTERVAL = pdMS_TO_TICKS(200);
const TickType_t WIFI_CHECK_INTERVAL = pdMS_TO_TICKS(100);
const uint32_t WIFI_RECONNECT_RETRY_MS = 5000;
const TickType_t FIREBASE_TASK_INTERVAL = pdMS_TO_TICKS(250);
const TickType_t FIREBASE_REALTIME_INTERVAL = pdMS_TO_TICKS(1000);
const TickType_t FIREBASE_HISTORY_INTERVAL = pdMS_TO_TICKS(300000);
const TickType_t FIREBASE_COMMAND_INTERVAL = pdMS_TO_TICKS(250);
const TickType_t FIREBASE_QUEUE_FLUSH_INTERVAL = pdMS_TO_TICKS(10000);
const TickType_t APPS_SCRIPT_INTERVAL = pdMS_TO_TICKS(60000);
const uint32_t WATCHDOG_TIMEOUT_SECONDS = 30;

SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;

FirebaseData fbdoRealtime;
FirebaseData fbdoHistory;
FirebaseData fbdoCommand;
FirebaseAuth auth;
FirebaseConfig config;

static std::vector<String> pendingFirebaseQueue;
const size_t MAX_FIREBASE_QUEUE = 288;
bool littleFsReady = false;
bool firebaseReadyLogged = false;
bool lastFirebaseKurasCommand = false;
bool firebaseKurasCommandClearPending = false;
TickType_t lastFirebaseRealtimeSend = 0;
TickType_t lastFirebaseCommandRead = 0;
TickType_t lastFirebaseHistoryPush = 0;
TickType_t lastFirebaseQueueFlush = 0;
TickType_t lastAppsScriptSend = 0;
bool appsScriptPernahDikirim = false;

// ================= PENGUJIAN KONEKTIVITAS IoT =================
// Statistik dihitung khusus untuk pengiriman payload sensor realtime.
uint32_t totalData = 0;
uint32_t dataBerhasil = 0;
uint32_t dataGagal = 0;
uint32_t reconnectCount = 0;
uint32_t reconnectTime = 0;
uint32_t reconnectStart = 0;
uint32_t reconnectEnd = 0;

uint64_t totalReconnectTime = 0;
int64_t totalRssi = 0;
uint32_t jumlahSampelRssi = 0;
uint32_t lastWiFiReconnectAttempt = 0;
bool wifiPernahTerhubung = false;
bool reconnectSedangBerlangsung = false;

enum StatusSistem {
  STATUS_INIT,
  STATUS_NORMAL,
  STATUS_SENSOR_ERROR,
  STATUS_ALIR_AWAL,
  STATUS_KURAS_TANDON,
  STATUS_SAFE_STOP,
  STATUS_KALIBRASI
};

StatusSistem statusSistem = STATUS_INIT;

bool sensorLevelAwalValid = false;
bool sensorLevelTandonValid = false;
bool sensorSuhuValid = false;
bool sensorPhValid = false;
uint8_t jsnTandonFailedReads = 0;
uint8_t jsnTandonJumpConfirmCount = 0;
float jsnTandonJumpCandidateCm = NAN;
bool sensorNonLevelPaused = false;
bool sensorNonLevelPumpWasActive = false;
uint32_t sensorNonLevelPumpOffMs = 0;
uint8_t sensorNonLevelSmoothReadsRemaining = 0;
bool airAwalPenuhLatch = false;
bool airTandonCukupLatch = false;

void serialPrintlnSafe(const String &text);
void resetFilterKeputusanTurbidityAwal();
void setInitialPump(bool aktif);
void setCleanSolenoid(bool aktif);
void setTurbidSolenoid(bool aktif);
bool solenoidRouteApplied(SolenoidRoute route);
void requestSolenoidRoute(SolenoidRoute route);
void processSolenoidTransition();
void publishActuatorStatus();

struct FirebasePayload {
  float kekeruhanAwal;
  float kekeruhanAwalRaw;
  float jarakAirAwal;
  float jarakAirAwalRaw;
  float adcTurbAwal;
  float voltTurbAwal;
  float voltTurbAwalRaw;
  float kekeruhanTandon;
  float kekeruhanTandonRaw;
  float phTandon;
  float suhuTandon;
  float jarakAirTandon;
  float adcTurbTandon;
  float voltTurbTandon;
  float voltTurbTandonRaw;
  float adcPhTandon;
  float voltPhTandon;
  bool sensorLevelAwalValid;
  bool sensorLevelTandonValid;
  bool sensorSuhuValid;
  bool sensorPhValid;
  bool airAwalPenuh;
  bool airTandonCukup;
  bool pumpAwal12vAktif;
  bool pumpKuras5vAktif;
  bool solenoidKeruhAktif;
  bool solenoidBersihAktif;
  bool perintahKurasTandon;
  StatusSistem status;
  uint32_t uptimeMs;
  int wifiRssi;
  size_t pendingQueueSize;
};

struct PengujianKonektivitasSnapshot {
  uint32_t totalData;
  uint32_t dataBerhasil;
  uint32_t dataGagal;
  uint32_t reconnectCount;
  uint32_t reconnectTime;
  uint32_t rataRataReconnect;
  int rataRataRssi;
  float persentaseKeberhasilan;
};

struct AnalogSensorReading {
  float adc;
  float volt;
  float nilai;
};

struct TurbidityRawReading {
  uint16_t adcRaw;
  float voltRaw;
  float adcAvg;
  float voltAvg;
  float adcEma;
  float ntu;
};

// =====================================================
// LCD FUNCTION
// =====================================================

void lcdRecovery() {
  Wire.end();
  delay(100);
  Wire.begin();
  delay(100);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("LCD RECOVERY");
  lcd.setCursor(0, 1);
  lcd.print("PLEASE WAIT");
  delay(500);

  lcd.clear();
}

void printLcdLine(byte row, String text) {
  text.concat("                ");

  lcd.setCursor(0, row);

  for (int i = 0; i < 16; i++) {
    lcd.print(text[i]);
  }
}

String formatJarak(float jarakCm) {
  if (isnan(jarakCm)) {
    return "ERR";
  }

  String jarakText(jarakCm, 1);
  jarakText.concat("cm");
  return jarakText;
}

float hitungTinggiAirAwal(float jarakSensorCm) {
  if (isnan(jarakSensorCm)) {
    return NAN;
  }

  float tinggiAirCm = TINGGI_KOSONG_PENAMPUNGAN_AWAL_CM - jarakSensorCm;

  if (tinggiAirCm < 0.0) {
    return 0.0;
  }

  if (tinggiAirCm > TINGGI_KOSONG_PENAMPUNGAN_AWAL_CM) {
    return TINGGI_KOSONG_PENAMPUNGAN_AWAL_CM;
  }

  return tinggiAirCm;
}

String statusSistemText(StatusSistem status) {
  switch (status) {
    case STATUS_INIT:
      return "INIT";
    case STATUS_NORMAL:
      return "NORMAL";
    case STATUS_SENSOR_ERROR:
      return "SENSOR ERR";
    case STATUS_ALIR_AWAL:
      return "ALIR AWAL";
    case STATUS_KURAS_TANDON:
      return "KURAS";
    case STATUS_SAFE_STOP:
      return "SAFE STOP";
    case STATUS_KALIBRASI:
      return "KALIBRASI";
    default:
      return "UNKNOWN";
  }
}

bool jarakValid(float jarakCm) {
  return !isnan(jarakCm) &&
         jarakCm >= JARAK_MIN_VALID_CM &&
         jarakCm <= JARAK_MAX_VALID_CM;
}

bool suhuValid(float suhuC) {
  return suhuC >= SUHU_MIN_VALID_C && suhuC <= SUHU_MAX_VALID_C;
}

bool phValid(float ph) {
  return ph >= PH_MIN_VALID && ph <= PH_MAX_VALID;
}

float filterSensorDenganAlpha(float nilaiLama,
                              float nilaiBaru,
                              bool nilaiLamaValid,
                              bool nilaiBaruValid,
                              float alpha) {
  if (!nilaiBaruValid) {
    return nilaiLama;
  }

  if (!nilaiLamaValid) {
    return nilaiBaru;
  }

  return (alpha * nilaiBaru) + ((1.0f - alpha) * nilaiLama);
}

float filterSensor(float nilaiLama, float nilaiBaru, bool nilaiLamaValid, bool nilaiBaruValid) {
  return filterSensorDenganAlpha(nilaiLama,
                                 nilaiBaru,
                                 nilaiLamaValid,
                                 nilaiBaruValid,
                                 FILTER_SENSOR_ALPHA);
}

void updatePauseSensorNonLevel() {
  bool pumpAktif = pumpAwal12vAktif || pumpKuras5vAktif;

  if (pumpAktif) {
    sensorNonLevelPaused = true;
    sensorNonLevelPumpWasActive = true;
    sensorNonLevelSmoothReadsRemaining = 0;
    return;
  }

  if (sensorNonLevelPumpWasActive) {
    sensorNonLevelPumpWasActive = false;
    sensorNonLevelPumpOffMs = millis();
    return;
  }

  if (sensorNonLevelPaused &&
      millis() - sensorNonLevelPumpOffMs >= SENSOR_NON_LEVEL_SETTLE_MS) {
    sensorNonLevelPaused = false;
    sensorNonLevelSmoothReadsRemaining = SENSOR_NON_LEVEL_SMOOTH_READS;
  }
}

void resetKonfirmasiLonjakanJsn() {
  jsnTandonJumpCandidateCm = NAN;
  jsnTandonJumpConfirmCount = 0;
}

void prosesPembacaanJsnTandon(float jarakBaruCm) {
  if (!jarakValid(jarakBaruCm)) {
    resetKonfirmasiLonjakanJsn();

    if (jsnTandonFailedReads < JSN_TANDON_MAX_FAILED_READS) {
      jsnTandonFailedReads++;
    }

    // Pertahankan nilai terakhir saat timeout singkat. Status baru menjadi
    // error jika beberapa batch berturut-turut sama sekali tidak menerima echo.
    if (jsnTandonFailedReads >= JSN_TANDON_MAX_FAILED_READS) {
      sensorLevelTandonValid = false;
    }
    return;
  }

  jsnTandonFailedReads = 0;

  if (!sensorLevelTandonValid) {
    jarak_air_tandon = jarakBaruCm;
    sensorLevelTandonValid = true;
    resetKonfirmasiLonjakanJsn();
    return;
  }

  if (fabsf(jarakBaruCm - jarak_air_tandon) > JSN_TANDON_MAX_SINGLE_JUMP_CM) {
    bool kandidatSama = !isnan(jsnTandonJumpCandidateCm) &&
                        fabsf(jarakBaruCm - jsnTandonJumpCandidateCm) <=
                            JSN_TANDON_JUMP_TOLERANCE_CM;

    if (!kandidatSama) {
      jsnTandonJumpCandidateCm = jarakBaruCm;
      jsnTandonJumpConfirmCount = 1;
      return;
    }

    jsnTandonJumpCandidateCm =
        (jsnTandonJumpCandidateCm + jarakBaruCm) * 0.5f;
    jsnTandonJumpConfirmCount++;

    if (jsnTandonJumpConfirmCount < JSN_TANDON_JUMP_CONFIRM_READS) {
      return;
    }

    // Perubahan besar baru diterapkan setelah muncul pada dua batch berurutan.
    jarak_air_tandon = jsnTandonJumpCandidateCm;
    sensorLevelTandonValid = true;
    resetKonfirmasiLonjakanJsn();
    return;
  }

  resetKonfirmasiLonjakanJsn();
  jarak_air_tandon = (JSN_TANDON_FILTER_ALPHA * jarakBaruCm) +
                     ((1.0f - JSN_TANDON_FILTER_ALPHA) * jarak_air_tandon);
  sensorLevelTandonValid = true;
}

float adcToVoltage(float adc) {
  return (adc / TURB_AWAL_ADC_MAX) * TURB_AWAL_VREF;
}

bool relayOutputAktif(uint8_t pin) {
  return digitalRead(pin) == RELAY_ON;
}

void inisialisasiRelayAman() {
  // Isi output latch dengan level OFF sebelum pin dijadikan OUTPUT.
  // Ini mencegah pulsa LOW sesaat yang dapat mengaktifkan relay active-low.
  digitalWrite(RELAY_PUMP_AWAL_12V_PIN, RELAY_OFF);
  digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_OFF);
  digitalWrite(RELAY_SOLENOID_KERUH_PIN, RELAY_OFF);
  digitalWrite(RELAY_SOLENOID_BERSIH_PIN, RELAY_OFF);

  pinMode(RELAY_PUMP_AWAL_12V_PIN, OUTPUT);
  pinMode(RELAY_PUMP_KURAS_5V_PIN, OUTPUT);
  pinMode(RELAY_SOLENOID_KERUH_PIN, OUTPUT);
  pinMode(RELAY_SOLENOID_BERSIH_PIN, OUTPUT);

  // Tulis ulang setelah mode OUTPUT sebagai pengaman tambahan.
  digitalWrite(RELAY_PUMP_AWAL_12V_PIN, RELAY_OFF);
  digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_OFF);
  digitalWrite(RELAY_SOLENOID_KERUH_PIN, RELAY_OFF);
  digitalWrite(RELAY_SOLENOID_BERSIH_PIN, RELAY_OFF);

  pumpAwal12vAktif = false;
  pumpKuras5vAktif = false;
  solenoidKeruhAktif = false;
  solenoidBersihAktif = false;
}

uint16_t readTurbiditySingle(uint8_t pin) {
  return (uint16_t)analogRead(pin);
}

float readTurbidityAverage(uint8_t pin, uint8_t sampleCount) {
  const uint8_t maxSamples = 64;
  uint8_t jumlahSampel = sampleCount;

  if (jumlahSampel == 0) {
    return analogRead(pin);
  }

  if (jumlahSampel > maxSamples) {
    jumlahSampel = maxSamples;
  }

  uint16_t samples[maxSamples];

  for (uint8_t i = 0; i < jumlahSampel; i++) {
    samples[i] = analogRead(pin);
    delayMicroseconds(700);
  }

  for (uint8_t i = 1; i < jumlahSampel; i++) {
    uint16_t nilai = samples[i];
    int8_t j = i - 1;

    while (j >= 0 && samples[j] > nilai) {
      samples[j + 1] = samples[j];
      j--;
    }

    samples[j + 1] = nilai;
  }

  uint8_t awal = jumlahSampel / 4;
  uint8_t akhir = jumlahSampel - awal;
  uint32_t sum = 0;
  uint8_t jumlahDipakai = 0;

  for (uint8_t i = awal; i < akhir; i++) {
    sum += samples[i];
    jumlahDipakai++;
  }

  if (jumlahDipakai == 0) {
    return samples[jumlahSampel / 2];
  }

  return (float)sum / jumlahDipakai;
}

uint16_t readTurbidityAwalSingle() {
  return readTurbiditySingle(TURB_AWAL_PIN);
}

float readTurbidityAwalAverage() {
  return readTurbidityAverage(TURB_AWAL_PIN, TURB_AWAL_AVG_SAMPLES);
}

float readTurbidityTandonAverage() {
  return readTurbidityAverage(TURB_TANDON_PIN, TURB_TANDON_AVG_SAMPLES);
}

bool turbidityAdcClearValid(float adcClear) {
  return !isnan(adcClear) && adcClear > 100.0 && adcClear <= TURB_AWAL_ADC_MAX;
}

float adcToTurbidityNtu(float adc,
                        float adcClear,
                        float zeroNtuBand,
                        float ntuGain,
                        bool bidirectionalAdc) {
  if (!turbidityAdcClearValid(adcClear)) {
    return 0;
  }

  float adcDelta = bidirectionalAdc ? fabs(adcClear - adc) : (adcClear - adc);

  if (adcDelta <= 0.0) {
    return 0.0;
  }

  float rawNtu = TURB_AWAL_NTU_MAX * (adcDelta / adcClear);

  if (rawNtu <= zeroNtuBand) {
    return 0.0;
  }

  float rentangEfektif = TURB_AWAL_NTU_MAX - zeroNtuBand;

  if (rentangEfektif <= 0.0) {
    return 0.0;
  }

  float ntu = ((rawNtu - zeroNtuBand) / rentangEfektif) *
              TURB_AWAL_NTU_MAX *
              ntuGain;

  if (ntu < 0.0) {
    return 0.0;
  }

  if (ntu > TURB_AWAL_NTU_MAX) {
    return TURB_AWAL_NTU_MAX;
  }

  return ntu;
}

float adcToTurbidityAwalNtu(float adc) {
  return adcToTurbidityNtu(adc,
                           turbAwalAdcClear,
                           TURB_AWAL_ZERO_NTU_BAND,
                           TURB_AWAL_NTU_GAIN,
                           TURB_AWAL_BIDIRECTIONAL_ADC);
}

float adcToTurbidityTandonNtu(float adc) {
  return adcToTurbidityNtu(adc,
                           turbTandonAdcClear,
                           TURB_TANDON_ZERO_NTU_BAND,
                           TURB_TANDON_NTU_GAIN,
                           TURB_TANDON_BIDIRECTIONAL_ADC);
}

float filterMedianTemporalTurbidityTandon(float adcBaru) {
  if (!turbTandonTemporalReady) {
    for (uint8_t i = 0; i < TURB_TANDON_TEMPORAL_SAMPLES; i++) {
      turbTandonTemporalAdc[i] = adcBaru;
    }

    turbTandonTemporalIndex = 0;
    turbTandonTemporalReady = true;
    return adcBaru;
  }

  turbTandonTemporalAdc[turbTandonTemporalIndex] = adcBaru;
  turbTandonTemporalIndex =
      (turbTandonTemporalIndex + 1) % TURB_TANDON_TEMPORAL_SAMPLES;

  float a = turbTandonTemporalAdc[0];
  float b = turbTandonTemporalAdc[1];
  float c = turbTandonTemporalAdc[2];

  if (a > b) {
    float sementara = a;
    a = b;
    b = sementara;
  }

  if (b > c) {
    float sementara = b;
    b = c;
    c = sementara;
  }

  if (a > b) {
    b = a;
  }

  return b;
}

TurbidityRawReading bacaTurbidityFinalRaw(uint8_t pin,
                                          float &emaAdc,
                                          bool &emaReady,
                                          float adcClear,
                                          float zeroNtuBand,
                                          float ntuGain,
                                          float emaAlphaLowerAdc,
                                          float emaAlphaHigherAdc,
                                          float maxAdcStep,
                                          bool bidirectionalAdc,
                                          uint8_t sampleCount) {
  TurbidityRawReading reading{};

  reading.adcRaw = readTurbiditySingle(pin);
  reading.adcAvg = readTurbidityAverage(pin, sampleCount);

  if (pin == TURB_TANDON_PIN) {
    reading.adcAvg = filterMedianTemporalTurbidityTandon(reading.adcAvg);
  }

  reading.voltRaw = adcToVoltage(reading.adcRaw);
  reading.voltAvg = adcToVoltage(reading.adcAvg);

  if (!emaReady) {
    emaAdc = reading.adcAvg;
    emaReady = true;
  } else {
    // Pada sensor ini ADC turun berarti air makin keruh. Alpha yang terpisah
    // memberi respons cepat ke kekeruhan tanpa mengorbankan kestabilan saat
    // nilai kembali menuju baseline air jernih.
    float emaAlpha = reading.adcAvg < emaAdc ?
                     emaAlphaLowerAdc : emaAlphaHigherAdc;
    float emaBaru = (emaAlpha * reading.adcAvg) +
                    ((1.0 - emaAlpha) * emaAdc);

    if (maxAdcStep > 0.0) {
      float deltaEma = emaBaru - emaAdc;

      if (deltaEma > maxAdcStep) {
        emaBaru = emaAdc + maxAdcStep;
      } else if (deltaEma < -maxAdcStep) {
        emaBaru = emaAdc - maxAdcStep;
      }
    }

    emaAdc = emaBaru;
  }

  reading.adcEma = emaAdc;
  reading.ntu = adcToTurbidityNtu(reading.adcEma,
                                  adcClear,
                                  zeroNtuBand,
                                  ntuGain,
                                  bidirectionalAdc);

  return reading;
}

TurbidityRawReading bacaTurbidityAwalRaw() {
  return bacaTurbidityFinalRaw(TURB_AWAL_PIN,
                               turbAwalEmaAdc,
                               turbAwalEmaReady,
                               turbAwalAdcClear,
                               TURB_AWAL_ZERO_NTU_BAND,
                               TURB_AWAL_NTU_GAIN,
                               TURB_AWAL_EMA_ALPHA_TURBID,
                               TURB_AWAL_EMA_ALPHA_CLEAR,
                               TURB_AWAL_MAX_ADC_STEP,
                               TURB_AWAL_BIDIRECTIONAL_ADC,
                               TURB_AWAL_AVG_SAMPLES);
}

TurbidityRawReading bacaTurbidityTandonRaw() {
  return bacaTurbidityFinalRaw(TURB_TANDON_PIN,
                               turbTandonEmaAdc,
                               turbTandonEmaReady,
                               turbTandonAdcClear,
                               TURB_TANDON_ZERO_NTU_BAND,
                               TURB_TANDON_NTU_GAIN,
                               TURB_TANDON_EMA_ALPHA_TURBID,
                               TURB_TANDON_EMA_ALPHA_CLEAR,
                               TURB_TANDON_MAX_ADC_STEP,
                               TURB_TANDON_BIDIRECTIONAL_ADC,
                               TURB_TANDON_AVG_SAMPLES);
}

void watchdogResetSafe() {
  esp_task_wdt_reset();
}

void registerTaskWatchdog() {
  esp_task_wdt_add(NULL);
  watchdogResetSafe();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("# Menghubungkan WiFi");
  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiPernahTerhubung = true;
    Serial.print("# WiFi terhubung. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("# WiFi belum terhubung. Sistem tetap berjalan offline.");
  }
}

void persistFirebaseQueue() {
  if (!SIMPAN_ANTREAN_FIREBASE || !littleFsReady) {
    return;
  }

  File qf = LittleFS.open("/firebase_queue.txt", "w");
  if (!qf) {
    return;
  }

  for (const auto &payload : pendingFirebaseQueue) {
    qf.println(payload);
  }

  qf.close();
}

void loadFirebaseQueue() {
  if (!SIMPAN_ANTREAN_FIREBASE || !littleFsReady || !LittleFS.exists("/firebase_queue.txt")) {
    return;
  }

  File qf = LittleFS.open("/firebase_queue.txt", "r");
  if (!qf) {
    return;
  }

  while (qf.available()) {
    String line = qf.readStringUntil('\n');
    line.trim();

    if (line.length() > 0 && pendingFirebaseQueue.size() < MAX_FIREBASE_QUEUE) {
      pendingFirebaseQueue.emplace_back(line);
    }
  }

  qf.close();

  Serial.print("Memuat antrean Firebase dari flash, item=");
  Serial.println(pendingFirebaseQueue.size());
}

void enqueueFailedFirebasePayload(FirebaseJson &json) {
  if (!SIMPAN_ANTREAN_FIREBASE) {
    return;
  }

  const char *raw = json.raw();

  if (!raw) {
    return;
  }

  if (pendingFirebaseQueue.size() >= MAX_FIREBASE_QUEUE) {
    pendingFirebaseQueue.erase(pendingFirebaseQueue.begin());
  }

  pendingFirebaseQueue.emplace_back(String(raw));
  persistFirebaseQueue();

  serialPrintlnSafe("Payload Firebase gagal, disimpan ke antrean lokal.");
}

FirebasePayload ambilFirebasePayloadSnapshot() {
  FirebasePayload payload{};

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    payload.kekeruhanAwal = kekeruhan_awal;
    payload.kekeruhanAwalRaw = kekeruhan_awal_raw;
    payload.jarakAirAwal = jarak_air_awal;
    payload.jarakAirAwalRaw = jarak_air_awal_raw;
    payload.adcTurbAwal = adc_turb_awal;
    payload.voltTurbAwal = volt_turb_awal;
    payload.voltTurbAwalRaw = volt_turb_awal_raw;
    payload.kekeruhanTandon = kekeruhan_tandon;
    payload.kekeruhanTandonRaw = kekeruhan_tandon_raw;
    payload.phTandon = ph_tandon;
    payload.suhuTandon = suhu_tandon;
    payload.jarakAirTandon = jarak_air_tandon;
    payload.adcTurbTandon = adc_turb_tandon;
    payload.voltTurbTandon = volt_turb_tandon;
    payload.voltTurbTandonRaw = volt_turb_tandon_raw;
    payload.adcPhTandon = adc_ph_tandon;
    payload.voltPhTandon = volt_ph_tandon;
    payload.sensorLevelAwalValid = sensorLevelAwalValid;
    payload.sensorLevelTandonValid = sensorLevelTandonValid;
    payload.sensorSuhuValid = sensorSuhuValid;
    payload.sensorPhValid = sensorPhValid;
    payload.airAwalPenuh = airAwalPenuhLatch;
    payload.airTandonCukup = airTandonCukupLatch;
    payload.pumpAwal12vAktif = relayOutputAktif(RELAY_PUMP_AWAL_12V_PIN);
    payload.pumpKuras5vAktif = relayOutputAktif(RELAY_PUMP_KURAS_5V_PIN);
    payload.solenoidKeruhAktif = relayOutputAktif(RELAY_SOLENOID_KERUH_PIN);
    payload.solenoidBersihAktif = relayOutputAktif(RELAY_SOLENOID_BERSIH_PIN);
    payload.perintahKurasTandon = perintahKurasTandon;
    payload.status = statusSistem;
    xSemaphoreGive(dataMutex);
  }

  payload.uptimeMs = millis();
  payload.wifiRssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  payload.pendingQueueSize = pendingFirebaseQueue.size();

  return payload;
}

void fillFirebasePayloadJson(FirebaseJson &json, const FirebasePayload &payload) {
  json.set("penampungan_awal/turbidity_sensor_raw", payload.kekeruhanAwalRaw);
  json.set("penampungan_awal/turbidity_adc_raw", payload.kekeruhanAwalRaw);
  json.set("penampungan_awal/turbidity_voltage_raw", payload.voltTurbAwalRaw);
  json.set("penampungan_awal/turbidity", payload.kekeruhanAwal);
  json.set("penampungan_awal/turbidity_adc", payload.adcTurbAwal);
  json.set("penampungan_awal/turbidity_voltage", payload.voltTurbAwal);
  json.set("penampungan_awal/turbidity_final/ntu", payload.kekeruhanAwal);
  json.set("penampungan_awal/turbidity_final/status",
           payload.kekeruhanAwal <= TURB_AWAL_NTU_THRESHOLD ? "JERNIH" : "KERUH");
  json.set("penampungan_awal/turbidity_final/adc_raw", payload.kekeruhanAwalRaw);
  json.set("penampungan_awal/turbidity_final/adc_ema", payload.adcTurbAwal);
  json.set("penampungan_awal/turbidity_final/volt_raw", payload.voltTurbAwalRaw);
  json.set("penampungan_awal/turbidity_final/volt_avg", payload.voltTurbAwal);
  json.set("penampungan_awal/jarak_sensor_cm_raw", payload.jarakAirAwalRaw);
  json.set("penampungan_awal/level_cm_sensor_raw", payload.jarakAirAwalRaw);
  json.set("penampungan_awal/level_cm_raw", payload.jarakAirAwal);
  json.set("penampungan_awal/level_cm", payload.sensorLevelAwalValid ? payload.jarakAirAwal : 0.0f);
  json.set("penampungan_awal/level_valid", payload.sensorLevelAwalValid);
  json.set("penampungan_awal/air_penuh", payload.airAwalPenuh);

  json.set("tandon_akhir/turbidity_sensor_raw", payload.kekeruhanTandonRaw);
  json.set("tandon_akhir/turbidity_adc_raw", payload.kekeruhanTandonRaw);
  json.set("tandon_akhir/turbidity_voltage_raw", payload.voltTurbTandonRaw);
  json.set("tandon_akhir/turbidity", payload.kekeruhanTandon);
  json.set("tandon_akhir/turbidity_adc", payload.adcTurbTandon);
  json.set("tandon_akhir/turbidity_voltage", payload.voltTurbTandon);
  json.set("tandon_akhir/turbidity_final/ntu", payload.kekeruhanTandon);
  json.set("tandon_akhir/turbidity_final/status",
           payload.kekeruhanTandon <= TURB_AWAL_NTU_THRESHOLD ? "JERNIH" : "KERUH");
  json.set("tandon_akhir/turbidity_final/adc_raw", payload.kekeruhanTandonRaw);
  json.set("tandon_akhir/turbidity_final/adc_ema", payload.adcTurbTandon);
  json.set("tandon_akhir/turbidity_final/volt_raw", payload.voltTurbTandonRaw);
  json.set("tandon_akhir/turbidity_final/volt_avg", payload.voltTurbTandon);
  json.set("tandon_akhir/ph_raw", payload.phTandon);
  json.set("tandon_akhir/ph", payload.sensorPhValid ? payload.phTandon : 0.0f);
  json.set("tandon_akhir/ph_adc", payload.adcPhTandon);
  json.set("tandon_akhir/ph_voltage", payload.voltPhTandon);
  json.set("tandon_akhir/ph_valid", payload.sensorPhValid);
  json.set("tandon_akhir/suhu_c_raw", payload.suhuTandon);
  json.set("tandon_akhir/suhu_c", payload.sensorSuhuValid ? payload.suhuTandon : 0.0f);
  json.set("tandon_akhir/suhu_valid", payload.sensorSuhuValid);
  json.set("tandon_akhir/level_cm_raw", payload.jarakAirTandon);
  json.set("tandon_akhir/level_cm", payload.sensorLevelTandonValid ? payload.jarakAirTandon : 0.0f);
  json.set("tandon_akhir/level_valid", payload.sensorLevelTandonValid);
  json.set("tandon_akhir/air_cukup_kuras", payload.airTandonCukup);

  json.set("aktuator/pump_awal_12v", payload.pumpAwal12vAktif);
  json.set("aktuator/pump_kuras_5v", payload.pumpKuras5vAktif);
  json.set("aktuator/solenoid_1_keruh", payload.solenoidKeruhAktif);
  json.set("aktuator/solenoid_2_bersih", payload.solenoidBersihAktif);
  json.set("aktuator/relay_in4_solenoid_bersih", payload.solenoidBersihAktif);
  json.set("aktuator/relay_in6_solenoid_keruh", payload.solenoidKeruhAktif);
  json.set("aktuator/relay_in1_pump_awal_12v", payload.pumpAwal12vAktif);
  json.set("aktuator/relay_in2_pump_kuras_5v", payload.pumpKuras5vAktif);
  json.set("aktuator/relay_in3_solenoid_bersih", payload.solenoidBersihAktif);
  json.set("aktuator/relay_in4_solenoid_keruh", payload.solenoidKeruhAktif);

  json.set("command/perintah_kuras_tandon", payload.perintahKurasTandon);
  json.set("kalibrasi/mode_aktif", MODE_KALIBRASI_SENSOR);
  json.set("kalibrasi/relay_dinonaktifkan", MODE_KALIBRASI_SENSOR);
  json.set("kalibrasi/firebase_history_aktif", FIREBASE_HISTORY_AKTIF);
  json.set("kalibrasi/queue_littlefs_aktif", SIMPAN_ANTREAN_FIREBASE);
  json.set("kalibrasi/level_awal_kosong_cm", TINGGI_KOSONG_PENAMPUNGAN_AWAL_CM);
  json.set("kalibrasi/level_awal_offset_cm", KALIBRASI_LEVEL_AWAL_OFFSET_CM);
  json.set("kalibrasi/turbidity_mode", "TRIMMED_AVG_TEMPORAL_MEDIAN_ASYMMETRIC_EMA");
  json.set("kalibrasi/turbidity_avg_samples", static_cast<int>(TURB_AWAL_AVG_SAMPLES));
  json.set("kalibrasi/turbidity_adc_max", TURB_AWAL_ADC_MAX);
  json.set("kalibrasi/turbidity_vref", TURB_AWAL_VREF);
  json.set("kalibrasi/turbidity_adc_clear", turbAwalAdcClear);
  json.set("kalibrasi/turbidity_ntu_max", TURB_AWAL_NTU_MAX);
  json.set("kalibrasi/turbidity_ntu_threshold", TURB_AWAL_NTU_THRESHOLD);
  json.set("kalibrasi/turbidity_zero_ntu_band", TURB_AWAL_ZERO_NTU_BAND);
  json.set("kalibrasi/turbidity_ema_alpha", TURB_AWAL_EMA_ALPHA_TURBID);
  json.set("kalibrasi/turbidity_awal_mode", "FAST_ATTACK_STABLE_RELEASE_TRIMMED_AVG");
  json.set("kalibrasi/turbidity_tandon_mode", "TRIMMED_AVG_MEDIAN3_RESPONSIVE_EMA");
  json.set("kalibrasi/turbidity_awal_avg_samples", static_cast<int>(TURB_AWAL_AVG_SAMPLES));
  json.set("kalibrasi/turbidity_awal_adc_clear", turbAwalAdcClear);
  json.set("kalibrasi/turbidity_awal_zero_ntu_band", TURB_AWAL_ZERO_NTU_BAND);
  json.set("kalibrasi/turbidity_awal_ntu_gain", TURB_AWAL_NTU_GAIN);
  // Field lama tetap dikirim agar dashboard yang sudah ada tidak rusak.
  json.set("kalibrasi/turbidity_awal_ema_alpha", TURB_AWAL_EMA_ALPHA_TURBID);
  json.set("kalibrasi/turbidity_awal_ema_alpha_turbid", TURB_AWAL_EMA_ALPHA_TURBID);
  json.set("kalibrasi/turbidity_awal_ema_alpha_clear", TURB_AWAL_EMA_ALPHA_CLEAR);
  json.set("kalibrasi/turbidity_awal_max_adc_step", TURB_AWAL_MAX_ADC_STEP);
  json.set("kalibrasi/turbidity_awal_high_value_latched", false);
  json.set("kalibrasi/turbidity_awal_keruh_confirm_reads",
           static_cast<int>(TURB_AWAL_KERUH_CONFIRM_READS));
  json.set("kalibrasi/turbidity_awal_pump_startup_ignore_ms",
           static_cast<int>(TURB_AWAL_PUMP_STARTUP_IGNORE_MS));
  json.set("kalibrasi/turbidity_tandon_avg_samples", static_cast<int>(TURB_TANDON_AVG_SAMPLES));
  json.set("kalibrasi/turbidity_tandon_temporal_median_samples",
           static_cast<int>(TURB_TANDON_TEMPORAL_SAMPLES));
  json.set("kalibrasi/turbidity_tandon_adc_clear", turbTandonAdcClear);
  json.set("kalibrasi/turbidity_tandon_zero_ntu_band", TURB_TANDON_ZERO_NTU_BAND);
  json.set("kalibrasi/turbidity_tandon_ntu_gain", TURB_TANDON_NTU_GAIN);
  // Field lama tetap dikirim agar dashboard yang sudah ada tidak rusak.
  json.set("kalibrasi/turbidity_tandon_ema_alpha", TURB_TANDON_EMA_ALPHA_TURBID);
  json.set("kalibrasi/turbidity_tandon_ema_alpha_turbid", TURB_TANDON_EMA_ALPHA_TURBID);
  json.set("kalibrasi/turbidity_tandon_ema_alpha_clear", TURB_TANDON_EMA_ALPHA_CLEAR);
  json.set("kalibrasi/ph_calibration_value", calibration_value);
  json.set("status/kode", static_cast<int>(payload.status));
  json.set("status/text", statusSistemText(payload.status));
  json.set("status/wifi_connected", WiFi.status() == WL_CONNECTED);
  json.set("status/wifi_rssi", payload.wifiRssi);
  json.set("status/pending_queue", static_cast<int>(payload.pendingQueueSize));
  json.set("status/millis", static_cast<int>(payload.uptimeMs));
}

void simpanHistoryPayloadLokal(const FirebasePayload &payload) {
  FirebaseJson json;
  fillFirebasePayloadJson(json, payload);
  enqueueFailedFirebasePayload(json);
  lastFirebaseHistoryPush = xTaskGetTickCount();
}

void fillFirebaseRealtimeJson(FirebaseJson &json, const FirebasePayload &payload) {
  json.set("penampungan_awal/turbidity_sensor_raw", payload.kekeruhanAwalRaw);
  json.set("penampungan_awal/turbidity_adc_raw", payload.kekeruhanAwalRaw);
  json.set("penampungan_awal/turbidity_voltage_raw", payload.voltTurbAwalRaw);
  json.set("penampungan_awal/turbidity", payload.kekeruhanAwal);
  json.set("penampungan_awal/turbidity_adc", payload.adcTurbAwal);
  json.set("penampungan_awal/turbidity_voltage", payload.voltTurbAwal);
  json.set("penampungan_awal/turbidity_final/ntu", payload.kekeruhanAwal);
  json.set("penampungan_awal/turbidity_final/status",
           payload.kekeruhanAwal <= TURB_AWAL_NTU_THRESHOLD ? "JERNIH" : "KERUH");
  json.set("penampungan_awal/turbidity_final/adc_raw", payload.kekeruhanAwalRaw);
  json.set("penampungan_awal/turbidity_final/adc_ema", payload.adcTurbAwal);
  json.set("penampungan_awal/turbidity_final/volt_raw", payload.voltTurbAwalRaw);
  json.set("penampungan_awal/turbidity_final/volt_avg", payload.voltTurbAwal);
  json.set("penampungan_awal/jarak_sensor_cm_raw", payload.jarakAirAwalRaw);
  json.set("penampungan_awal/level_cm_sensor_raw", payload.jarakAirAwalRaw);
  json.set("penampungan_awal/level_cm_raw", payload.jarakAirAwal);
  json.set("penampungan_awal/level_cm", payload.sensorLevelAwalValid ? payload.jarakAirAwal : 0.0f);
  json.set("penampungan_awal/level_valid", payload.sensorLevelAwalValid);
  json.set("penampungan_awal/air_penuh", payload.airAwalPenuh);

  json.set("tandon_akhir/turbidity_sensor_raw", payload.kekeruhanTandonRaw);
  json.set("tandon_akhir/turbidity_adc_raw", payload.kekeruhanTandonRaw);
  json.set("tandon_akhir/turbidity_voltage_raw", payload.voltTurbTandonRaw);
  json.set("tandon_akhir/turbidity", payload.kekeruhanTandon);
  json.set("tandon_akhir/turbidity_adc", payload.adcTurbTandon);
  json.set("tandon_akhir/turbidity_voltage", payload.voltTurbTandon);
  json.set("tandon_akhir/turbidity_final/ntu", payload.kekeruhanTandon);
  json.set("tandon_akhir/turbidity_final/status",
           payload.kekeruhanTandon <= TURB_AWAL_NTU_THRESHOLD ? "JERNIH" : "KERUH");
  json.set("tandon_akhir/turbidity_final/adc_raw", payload.kekeruhanTandonRaw);
  json.set("tandon_akhir/turbidity_final/adc_ema", payload.adcTurbTandon);
  json.set("tandon_akhir/turbidity_final/volt_raw", payload.voltTurbTandonRaw);
  json.set("tandon_akhir/turbidity_final/volt_avg", payload.voltTurbTandon);
  json.set("tandon_akhir/ph_raw", payload.phTandon);
  json.set("tandon_akhir/ph", payload.sensorPhValid ? payload.phTandon : 0.0f);
  json.set("tandon_akhir/ph_adc", payload.adcPhTandon);
  json.set("tandon_akhir/ph_voltage", payload.voltPhTandon);
  json.set("tandon_akhir/ph_valid", payload.sensorPhValid);
  json.set("tandon_akhir/suhu_c_raw", payload.suhuTandon);
  json.set("tandon_akhir/suhu_c", payload.sensorSuhuValid ? payload.suhuTandon : 0.0f);
  json.set("tandon_akhir/suhu_valid", payload.sensorSuhuValid);
  json.set("tandon_akhir/level_cm_raw", payload.jarakAirTandon);
  json.set("tandon_akhir/level_cm", payload.sensorLevelTandonValid ? payload.jarakAirTandon : 0.0f);
  json.set("tandon_akhir/level_valid", payload.sensorLevelTandonValid);
  json.set("tandon_akhir/air_cukup_kuras", payload.airTandonCukup);

  json.set("aktuator/pump_awal_12v", payload.pumpAwal12vAktif);
  json.set("aktuator/pump_kuras_5v", payload.pumpKuras5vAktif);
  json.set("aktuator/solenoid_1_keruh", payload.solenoidKeruhAktif);
  json.set("aktuator/solenoid_2_bersih", payload.solenoidBersihAktif);
  json.set("aktuator/relay_in4_solenoid_bersih", payload.solenoidBersihAktif);
  json.set("aktuator/relay_in6_solenoid_keruh", payload.solenoidKeruhAktif);
  json.set("aktuator/relay_in1_pump_awal_12v", payload.pumpAwal12vAktif);
  json.set("aktuator/relay_in2_pump_kuras_5v", payload.pumpKuras5vAktif);
  json.set("aktuator/relay_in3_solenoid_bersih", payload.solenoidBersihAktif);
  json.set("aktuator/relay_in4_solenoid_keruh", payload.solenoidKeruhAktif);

  json.set("command/perintah_kuras_tandon", payload.perintahKurasTandon);
  json.set("status/kode", static_cast<int>(payload.status));
  json.set("status/text", statusSistemText(payload.status));
  json.set("status/wifi_connected", WiFi.status() == WL_CONNECTED);
  json.set("status/wifi_rssi", payload.wifiRssi);
  json.set("status/pending_queue", static_cast<int>(payload.pendingQueueSize));
  json.set("status/millis", static_cast<int>(payload.uptimeMs));
}

void tampilHasilPengujianKonektivitas(
    const PengujianKonektivitasSnapshot &hasil,
    bool firebaseBerhasil,
    int rssi,
    bool tampilkanRingkasan) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  Serial.println("==================================================");
  Serial.println("PENGUJIAN KONEKTIVITAS IoT");
  Serial.println("==========================");
  Serial.println();
  Serial.print("Data terkirim      : ");
  Serial.println(hasil.totalData);
  Serial.print("Data berhasil      : ");
  Serial.println(hasil.dataBerhasil);
  Serial.print("Data gagal         : ");
  Serial.println(hasil.dataGagal);
  Serial.print("Keberhasilan       : ");

  float persentaseBulat = roundf(hasil.persentaseKeberhasilan);
  if (fabsf(hasil.persentaseKeberhasilan - persentaseBulat) < 0.005f) {
    Serial.print(static_cast<int>(persentaseBulat));
  } else {
    Serial.print(hasil.persentaseKeberhasilan, 2);
  }

  Serial.println(" %");
  Serial.print("Reconnect ke       : ");
  Serial.println(hasil.reconnectCount);
  Serial.print("Waktu reconnect    : ");
  Serial.print(hasil.reconnectTime);
  Serial.println(" ms");
  Serial.print("RSSI WiFi          : ");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.print("Status Firebase    : ");
  Serial.println(firebaseBerhasil ? "BERHASIL" : "GAGAL");
  Serial.println("===========================");

  if (tampilkanRingkasan) {
    Serial.println("================ HASIL PENGUJIAN ================");
    Serial.print("Total Data Dikirim    : ");
    Serial.println(hasil.totalData);
    Serial.print("Data Berhasil         : ");
    Serial.println(hasil.dataBerhasil);
    Serial.print("Data Gagal            : ");
    Serial.println(hasil.dataGagal);
    Serial.print("Keberhasilan          : ");
    Serial.print(hasil.persentaseKeberhasilan, 2);
    Serial.println(" %");
    Serial.print("Jumlah Reconnect      : ");
    Serial.println(hasil.reconnectCount);
    Serial.print("Rata-rata Reconnect   : ");
    Serial.print(hasil.rataRataReconnect);
    Serial.println(" ms");
    Serial.print("RSSI Rata-rata        : ");
    Serial.print(hasil.rataRataRssi);
    Serial.println(" dBm");
    Serial.println("==================================================");
  }

  xSemaphoreGive(serialMutex);
}

void catatHasilPengirimanFirebase(bool firebaseBerhasil, int rssi) {
  PengujianKonektivitasSnapshot hasil{};
  bool tampilkanRingkasan = false;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  totalData++;

  if (firebaseBerhasil) {
    dataBerhasil++;
  } else {
    dataGagal++;
  }

  if (rssi != 0) {
    totalRssi += rssi;
    jumlahSampelRssi++;
  }

  hasil.totalData = totalData;
  hasil.dataBerhasil = dataBerhasil;
  hasil.dataGagal = dataGagal;
  hasil.reconnectCount = reconnectCount;
  hasil.reconnectTime = reconnectTime;
  hasil.persentaseKeberhasilan = totalData > 0
      ? (static_cast<float>(dataBerhasil) / static_cast<float>(totalData)) * 100.0f
      : 0.0f;
  hasil.rataRataReconnect = reconnectCount > 0
      ? static_cast<uint32_t>(totalReconnectTime / reconnectCount)
      : 0;
  hasil.rataRataRssi = jumlahSampelRssi > 0
      ? static_cast<int>(totalRssi / jumlahSampelRssi)
      : 0;
  tampilkanRingkasan = totalData % 100 == 0;

  xSemaphoreGive(dataMutex);

  tampilHasilPengujianKonektivitas(
      hasil, firebaseBerhasil, rssi, tampilkanRingkasan);
}

bool sendRealtimePayloadToFirebase(const FirebasePayload &payload) {
  FirebaseJson json;
  fillFirebaseRealtimeJson(json, payload);

  // Versi sinkron diperlukan agar nilai BERHASIL benar-benar merupakan
  // respons dari Firebase, bukan hanya request yang masuk antrean async.
  bool berhasil = Firebase.RTDB.updateNodeSilent(
      &fbdoRealtime, FIREBASE_REALTIME_PATH, &json);
  int rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0;
  catatHasilPengirimanFirebase(berhasil, rssi);

  if (berhasil) {
    return true;
  }

  String pesan("Gagal kirim realtime Firebase: ");
  pesan.concat(fbdoRealtime.errorReason());
  serialPrintlnSafe(pesan);
  return false;
}

void publishActuatorStatus() {
  bool publishNeeded = false;
  bool pumpAktif = false;
  bool solenoidBersih = false;
  bool solenoidKeruh = false;
  uint32_t generation = 0;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    publishNeeded = actuatorStatusDirty;
    generation = actuatorStatusGeneration;
    pumpAktif = relayOutputAktif(RELAY_PUMP_AWAL_12V_PIN);
    solenoidBersih = relayOutputAktif(RELAY_SOLENOID_BERSIH_PIN);
    solenoidKeruh = relayOutputAktif(RELAY_SOLENOID_KERUH_PIN);
    xSemaphoreGive(dataMutex);
  }

  if (!publishNeeded) {
    return;
  }

  FirebaseJson json;
  json.set("pump_awal_12v", pumpAktif);
  json.set("relay_in4_solenoid_bersih", solenoidBersih);
  json.set("relay_in6_solenoid_keruh", solenoidKeruh);
  json.set("solenoid_1_keruh", solenoidKeruh);
  json.set("solenoid_2_bersih", solenoidBersih);
  // Field lama tetap dipertahankan untuk instalasi yang sudah menggunakannya.
  json.set("relay_in1_pump_awal_12v", pumpAktif);
  json.set("relay_in3_solenoid_bersih", solenoidBersih);
  json.set("relay_in4_solenoid_keruh", solenoidKeruh);

  if (!Firebase.RTDB.updateNodeSilentAsync(
          &fbdoRealtime, "/sistem_air/realtime/aktuator", &json)) {
    return;
  }

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    // Jangan menghapus dirty flag jika output berubah lagi saat request dikirim.
    if (actuatorStatusGeneration == generation) {
      actuatorStatusDirty = false;
    }
    xSemaphoreGive(dataMutex);
  }
}

bool pushHistoryPayloadToFirebase(const FirebasePayload &payload) {
  FirebaseJson json;
  fillFirebasePayloadJson(json, payload);

  if (Firebase.RTDB.pushJSONAsync(&fbdoHistory, FIREBASE_HISTORY_PATH, &json)) {
    lastFirebaseHistoryPush = xTaskGetTickCount();
    return true;
  }

  String pesan("Gagal kirim history Firebase: ");
  pesan.concat(fbdoHistory.errorReason());
  serialPrintlnSafe(pesan);
  enqueueFailedFirebasePayload(json);
  lastFirebaseHistoryPush = xTaskGetTickCount();
  return false;
}

bool kirimSensorKeSpreadsheet(const FirebasePayload &payload) {
  if (WiFi.status() != WL_CONNECTED ||
      !payload.sensorPhValid ||
      !payload.sensorSuhuValid) {
    return false;
  }

  String url(APPS_SCRIPT_URL);
  url.reserve(url.length() + 96);
  url.concat("?ph=");
  url.concat(String(payload.phTandon, 2));
  url.concat("&suhu=");
  url.concat(String(payload.suhuTandon, 2));
  url.concat("&turbidity=");
  url.concat(String(payload.kekeruhanTandon, 2));

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

  if (!http.begin(secureClient, url)) {
    serialPrintlnSafe("Gagal memulai koneksi Google Apps Script.");
    return false;
  }

  int httpCode = http.GET();
  http.end();

  if (httpCode >= 200 && httpCode < 400) {
    serialPrintlnSafe("Data pH, suhu, dan turbidity terkirim ke spreadsheet.");
    return true;
  }

  String pesan("Gagal kirim data ke spreadsheet. HTTP code: ");
  pesan.concat(httpCode);
  serialPrintlnSafe(pesan);
  return false;
}

void flushPendingFirebaseQueue() {
  if (!SIMPAN_ANTREAN_FIREBASE || pendingFirebaseQueue.empty() || !Firebase.ready()) {
    return;
  }

  String payload = pendingFirebaseQueue.front();
  FirebaseJson json;
  json.setJsonData(payload.c_str());

  if (Firebase.RTDB.pushJSONAsync(&fbdoHistory, FIREBASE_HISTORY_PATH, &json)) {
    pendingFirebaseQueue.erase(pendingFirebaseQueue.begin());
    persistFirebaseQueue();
    serialPrintlnSafe("Flush antrean Firebase: item terkirim ke history.");
  }
}

void setKurasTandonManual(bool aktif, const char *sumber) {
  perintahKurasTandon = aktif;

  if (aktif) {
    if (!pumpKuras5vAktif) {
      digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_ON);
      pumpKuras5vAktif = true;
      triggerKurasTandon = true;
      statusSistem = STATUS_KURAS_TANDON;
    }

    String pesan(sumber);
    pesan.concat(": KURAS_ON");
    serialPrintlnSafe(pesan);
    return;
  }

  digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_OFF);
  pumpKuras5vAktif = false;
  triggerKurasTandon = false;
  relayBaruOff = true;

  if (statusSistem == STATUS_KURAS_TANDON) {
    statusSistem = STATUS_NORMAL;
  }

  String pesan(sumber);
  pesan.concat(": KURAS_OFF");
  serialPrintlnSafe(pesan);
}

bool readJsonBoolean(FirebaseJson &json, const char *path, bool &value) {
  FirebaseJsonData data;
  json.get(data, path);

  if (!data.success || data.typeNum != FirebaseJson::JSON_BOOL) {
    return false;
  }

  value = data.boolValue;
  return true;
}

bool readJsonSolenoidRoute(FirebaseJson &json,
                           const char *path,
                           SolenoidRoute &route) {
  FirebaseJsonData data;
  json.get(data, path);

  if (!data.success || data.typeNum != FirebaseJson::JSON_STRING) {
    return false;
  }

  String routeText = data.stringValue;
  routeText.trim();
  routeText.toLowerCase();

  if (routeText == "bersih") {
    route = SOLENOID_ROUTE_CLEAN;
    return true;
  }

  if (routeText == "keruh") {
    route = SOLENOID_ROUTE_TURBID;
    return true;
  }

  route = SOLENOID_ROUTE_NONE;
  return false;
}

void setFirebaseControlConnected(bool connected) {
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    firebaseControlConnected = connected;
    xSemaphoreGive(dataMutex);
  }
}

void readControlCommands() {
  if (!Firebase.ready() ||
      xTaskGetTickCount() - lastFirebaseCommandRead < FIREBASE_COMMAND_INTERVAL) {
    return;
  }

  lastFirebaseCommandRead = xTaskGetTickCount();

  if (!Firebase.RTDB.getJSON(&fbdoCommand, FIREBASE_COMMANDS_PATH)) {
    setFirebaseControlConnected(false);
    return;
  }

  FirebaseJson *json = fbdoCommand.to<FirebaseJson *>();

  if (json == nullptr) {
    setFirebaseControlConnected(false);
    return;
  }

  bool modeManual = false;
  bool pumpAwal = false;
  bool kurasTandon = false;
  SolenoidRoute route = SOLENOID_ROUTE_NONE;
  bool modeValid = readJsonBoolean(*json, "mode_manual", modeManual);
  bool pumpValid = readJsonBoolean(*json, "pump_awal_12v", pumpAwal);
  bool routeValid = readJsonSolenoidRoute(*json, "jalur_solenoid", route);
  bool kurasValid = readJsonBoolean(*json, "kuras_tandon", kurasTandon);

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    commandModeManual = modeManual;
    commandModeManualValid = modeValid;
    commandPumpAwal12v = pumpAwal;
    commandPumpAwal12vValid = pumpValid;
    commandSolenoidRoute = route;
    commandSolenoidRouteValid = routeValid;
    firebaseControlConnected = true;

    if (kurasValid) {
      bool perluTerapkan = kurasTandon != lastFirebaseKurasCommand ||
                           (kurasTandon && !pumpKuras5vAktif) ||
                           (!kurasTandon &&
                            (perintahKurasTandon || pumpKuras5vAktif || triggerKurasTandon));

      if (perluTerapkan) {
        lastFirebaseKurasCommand = kurasTandon;
        setKurasTandonManual(kurasTandon, "Firebase command");
      }
    }

    xSemaphoreGive(dataMutex);
  }
}

bool resetInitialPumpCommandAfterConnect() {
  if (!firebaseInitialPumpResetPending) {
    return false;
  }

  if (!Firebase.RTDB.setBool(&fbdoCommand, FIREBASE_PUMP_AWAL_PATH, false)) {
    String pesan("Gagal reset command pump awal Firebase: ");
    pesan.concat(fbdoCommand.errorReason());
    serialPrintlnSafe(pesan);
    return true;
  }

  firebaseInitialPumpResetPending = false;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    commandPumpAwal12v = false;
    commandPumpAwal12vValid = true;
    xSemaphoreGive(dataMutex);
  }

  serialPrintlnSafe("Command pump_awal_12v direset OFF setelah boot.");
  return false;
}

bool resetPerintahKurasFirebaseJikaPerlu() {
  bool resetPending = false;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    resetPending = firebaseKurasCommandClearPending;
    xSemaphoreGive(dataMutex);
  }

  if (!resetPending) {
    return false;
  }

  if (Firebase.RTDB.setBool(&fbdoCommand, FIREBASE_COMMAND_PATH, false)) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      firebaseKurasCommandClearPending = false;
      setKurasTandonManual(false, "Firebase command otomatis");
      xSemaphoreGive(dataMutex);
    }

    lastFirebaseKurasCommand = false;
    return false;
  }

  String pesan("Gagal reset command kuras Firebase: ");
  pesan.concat(fbdoCommand.errorReason());
  serialPrintlnSafe(pesan);
  return true;
}

void initFirebase() {
  if (SIMPAN_ANTREAN_FIREBASE) {
    littleFsReady = LittleFS.begin(true);

    if (!littleFsReady) {
      Serial.println("# LittleFS mount gagal. Antrean Firebase tidak disimpan setelah reboot.");
    } else {
      loadFirebaseQueue();
    }
  } else {
    littleFsReady = false;
    Serial.println("# Mode kalibrasi: antrean Firebase LittleFS dinonaktifkan.");
  }

  connectWiFi();

  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  config.token_status_callback = tokenStatusCallback;
  config.timeout.socketConnection = 5 * 1000;
  config.timeout.serverResponse = 5 * 1000;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  fbdoRealtime.setResponseSize(512);
  fbdoHistory.setResponseSize(2048);
  fbdoCommand.setResponseSize(512);
  fbdoRealtime.setBSSLBufferSize(4096, 1024);
  fbdoHistory.setBSSLBufferSize(4096, 4096);
  fbdoCommand.setBSSLBufferSize(2048, 1024);

  Serial.println("# Firebase init selesai. Menunggu koneksi siap...");
}

void serialPrintlnSafe(const String &text) {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(text);
    xSemaphoreGive(serialMutex);
  }
}

void saveTurbidityAdcClear() {
  EEPROM.write(TURB_AWAL_ADDR_MAGIC, TURB_AWAL_EEPROM_MAGIC);
  EEPROM.put(TURB_AWAL_ADDR_ADC_CLEAR, turbAwalAdcClear);
  EEPROM.put(TURB_TANDON_ADDR_ADC_CLEAR, turbTandonAdcClear);
  EEPROM.commit();
}

void loadTurbidityAdcClear() {
  if (EEPROM.read(TURB_AWAL_ADDR_MAGIC) == TURB_AWAL_EEPROM_MAGIC) {
    EEPROM.get(TURB_AWAL_ADDR_ADC_CLEAR, turbAwalAdcClear);
    EEPROM.get(TURB_TANDON_ADDR_ADC_CLEAR, turbTandonAdcClear);
    bool nilaiDiperbaiki = false;

    if (!turbidityAdcClearValid(turbAwalAdcClear)) {
      turbAwalAdcClear = TURB_AWAL_DEFAULT_ADC_CLEAR;
      nilaiDiperbaiki = true;
    }

    if (!turbidityAdcClearValid(turbTandonAdcClear)) {
      turbTandonAdcClear = TURB_TANDON_DEFAULT_ADC_CLEAR;
      nilaiDiperbaiki = true;
    }

    if (nilaiDiperbaiki) {
      saveTurbidityAdcClear();
    }

    Serial.print("[EEPROM] ADC_CLEAR awal dimuat: ");
    Serial.println((int)turbAwalAdcClear);
    Serial.print("[EEPROM] ADC_CLEAR tandon dimuat: ");
    Serial.println((int)turbTandonAdcClear);
  } else {
    turbAwalAdcClear = TURB_AWAL_DEFAULT_ADC_CLEAR;
    turbTandonAdcClear = TURB_TANDON_DEFAULT_ADC_CLEAR;
    saveTurbidityAdcClear();
    Serial.print("[EEPROM] Pakai default ADC_CLEAR awal: ");
    Serial.println((int)turbAwalAdcClear);
    Serial.print("[EEPROM] Pakai default ADC_CLEAR tandon: ");
    Serial.println((int)turbTandonAdcClear);
  }
}

void printTurbidityAwalInfo() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println("+-------------------------------------+");
    Serial.println("|      PARAMETER KALIBRASI AKTIF      |");
    Serial.println("+-------------------------------------+");
    Serial.print("| ADC_CLEAR awal          : ");
    Serial.println((int)turbAwalAdcClear);
    Serial.print("| ADC_CLEAR tandon        : ");
    Serial.println((int)turbTandonAdcClear);
    Serial.print("| NTU_MAX   (air keruh)   : ");
    Serial.println((int)TURB_AWAL_NTU_MAX);
    Serial.print("| Threshold bersih/keruh  : ");
    Serial.print(TURB_AWAL_NTU_THRESHOLD, 0);
    Serial.println(" NTU");
    Serial.print("| Zero band awal          : ");
    Serial.print(TURB_AWAL_ZERO_NTU_BAND, 0);
    Serial.println(" NTU");
    Serial.print("| Sample awal             : ");
    Serial.println((int)TURB_AWAL_AVG_SAMPLES);
    Serial.print("| Gain awal               : ");
    Serial.println(TURB_AWAL_NTU_GAIN, 2);
    Serial.print("| EMA awal makin keruh    : ");
    Serial.println(TURB_AWAL_EMA_ALPHA_TURBID, 2);
    Serial.print("| EMA awal makin jernih   : ");
    Serial.println(TURB_AWAL_EMA_ALPHA_CLEAR, 2);
    Serial.print("| Max step ADC awal       : ");
    Serial.println(TURB_AWAL_MAX_ADC_STEP, 0);
    Serial.print("| Confirm keruh awal      : ");
    Serial.println((int)TURB_AWAL_KERUH_CONFIRM_READS);
    Serial.print("| Ignore pump startup     : ");
    Serial.print(TURB_AWAL_PUMP_STARTUP_IGNORE_MS / 1000);
    Serial.println(" detik");
    Serial.print("| Zero band tandon        : ");
    Serial.print(TURB_TANDON_ZERO_NTU_BAND, 0);
    Serial.println(" NTU");
    Serial.print("| Sample tandon           : ");
    Serial.println((int)TURB_TANDON_AVG_SAMPLES);
    Serial.print("| Median waktu tandon     : ");
    Serial.println((int)TURB_TANDON_TEMPORAL_SAMPLES);
    Serial.print("| Gain tandon             : ");
    Serial.println(TURB_TANDON_NTU_GAIN, 2);
    Serial.print("| EMA tandon makin keruh  : ");
    Serial.println(TURB_TANDON_EMA_ALPHA_TURBID, 2);
    Serial.print("| EMA tandon makin jernih : ");
    Serial.println(TURB_TANDON_EMA_ALPHA_CLEAR, 2);
    Serial.println("|                                     |");
    Serial.println("| Rumus: deadband + inverse linear   |");
    Serial.println("| Awal re-clear, tandon inverse      |");
    Serial.println("+-------------------------------------+");
    Serial.println("| Awal: angka / C=auto clear          |");
    Serial.println("| Tandon: TA=4000 / CT=auto clear     |");
    Serial.println("| I=info  T=raw ADC                   |");
    Serial.println("+-------------------------------------+");
    xSemaphoreGive(serialMutex);
  }
}

void printTurbidityAwalBanner() {
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println();
    Serial.println("+======================================+");
    Serial.println("|   TURBIDITY SENSOR - KALIBRASI FINAL |");
    Serial.println("|   GPIO 34 | 5V sensor | 3.3V ESP32   |");
    Serial.println("+======================================+");
    xSemaphoreGive(serialMutex);
  }
}

String statusJalurAwal(bool solenoidKeruh, bool solenoidBersih) {
  if (solenoidKeruh) {
    return "KERUH";
  }

  if (solenoidBersih) {
    return "BERSIH";
  }

  return "IDLE";
}

String gabung2(const char *a, const String &b) {
  String text(a);
  text.concat(b);
  return text;
}

String gabung3(const char *a, const String &b, const char *c) {
  String text(a);
  text.concat(b);
  text.concat(c);
  return text;
}

String gabung4(const char *a, const String &b, const char *c, const String &d) {
  String text(a);
  text.concat(b);
  text.concat(c);
  text.concat(d);
  return text;
}

void tampilSensorLCD(byte page,
                     float kekeruhanAwal,
                     float jarakAwal,
                     float kekeruhanTandon,
                     float phTandon,
                     float suhuTandon,
                     float jarakTandon,
                     bool pumpAwalAktif,
                     bool pumpKurasAktif,
                     bool solenoidKeruh,
                     bool solenoidBersih,
                     bool perintahKuras,
                     StatusSistem status,
                     bool levelAwalValid,
                     bool levelTandonValid) {
  if (page == 0) {
    printLcdLine(0, gabung2("AW T:", String((int)kekeruhanAwal)));
    printLcdLine(1, levelAwalValid ? gabung2("AW L:", formatJarak(jarakAwal)) : String("AW L:ERR"));
  } else if (page == 1) {
    printLcdLine(0, gabung2("TD T:", String((int)kekeruhanTandon)));
    printLcdLine(1, levelTandonValid ? gabung2("TD L:", formatJarak(jarakTandon)) : String("TD L:ERR"));
  } else if (page == 2) {
    printLcdLine(0, gabung2("pH:", String(phTandon, 2)));
    printLcdLine(1, gabung3("Suhu:", String(suhuTandon, 1), "C"));
  } else if (page == 3) {
    if (MODE_KALIBRASI_SENSOR) {
      printLcdLine(0, "MODE KALIBRASI");
      printLcdLine(1, "RELAY OFF");
    } else {
      printLcdLine(0, gabung2("Jalur:", statusJalurAwal(solenoidKeruh, solenoidBersih)));
      printLcdLine(1, gabung4("P12:",
                              String(pumpAwalAktif ? "ON" : "OFF"),
                              " K5:",
                              String(pumpKurasAktif ? "ON" : "OFF")));
    }
  } else {
    printLcdLine(0, "Status Sistem");
    printLcdLine(1, statusSistemText(status));
  }
}

void updateLCD() {
  byte pageSnapshot = 0;
  float kekeruhanAwalSnapshot = 0;
  float jarakAwalSnapshot = 0;
  float kekeruhanTandonSnapshot = 0;
  float phTandonSnapshot = 0;
  float suhuTandonSnapshot = 0;
  float jarakTandonSnapshot = 0;
  bool pumpAwalSnapshot = false;
  bool pumpKurasSnapshot = false;
  bool solenoidKeruhSnapshot = false;
  bool solenoidBersihSnapshot = false;
  bool perintahKurasSnapshot = false;
  bool levelAwalValidSnapshot = false;
  bool levelTandonValidSnapshot = false;
  StatusSistem statusSnapshot = STATUS_INIT;
  bool perluRecovery = false;
  bool perluTampil = false;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  // Saat relay aktif, LCD tidak dikirimi data supaya komunikasi I2C
  // tidak mudah terganggu noise dari relay/pompa/solenoid.
  if (pumpAwal12vAktif || pumpKuras5vAktif || solenoidKeruhAktif || solenoidBersihAktif) {
    xSemaphoreGive(dataMutex);
    return;
  }

  if (relayBaruOff) {
    relayBaruOff = false;
    lastLcdUpdate = millis();
    perluRecovery = true;
    perluTampil = true;
  } else if (millis() - lastLcdUpdate >= LCD_INTERVAL) {
    lastLcdUpdate = millis();
    lcdPage = (lcdPage + 1) % 5;
    perluTampil = true;
  }

  if (perluTampil) {
    pageSnapshot = lcdPage;
    kekeruhanAwalSnapshot = kekeruhan_awal;
    jarakAwalSnapshot = jarak_air_awal;
    kekeruhanTandonSnapshot = kekeruhan_tandon;
    phTandonSnapshot = ph_tandon;
    suhuTandonSnapshot = suhu_tandon;
    jarakTandonSnapshot = jarak_air_tandon;
    pumpAwalSnapshot = pumpAwal12vAktif;
    pumpKurasSnapshot = pumpKuras5vAktif;
    solenoidKeruhSnapshot = solenoidKeruhAktif;
    solenoidBersihSnapshot = solenoidBersihAktif;
    perintahKurasSnapshot = perintahKurasTandon;
    statusSnapshot = statusSistem;
    levelAwalValidSnapshot = sensorLevelAwalValid;
    levelTandonValidSnapshot = sensorLevelTandonValid;
  }

  xSemaphoreGive(dataMutex);

  if (!perluTampil) {
    return;
  }

  if (perluRecovery) {
    lcdRecovery();
  }

  tampilSensorLCD(pageSnapshot,
                  kekeruhanAwalSnapshot,
                  jarakAwalSnapshot,
                  kekeruhanTandonSnapshot,
                  phTandonSnapshot,
                  suhuTandonSnapshot,
                  jarakTandonSnapshot,
                  pumpAwalSnapshot,
                  pumpKurasSnapshot,
                  solenoidKeruhSnapshot,
                  solenoidBersihSnapshot,
                  perintahKurasSnapshot,
                  statusSnapshot,
                  levelAwalValidSnapshot,
                  levelTandonValidSnapshot);
}

// =====================================================
// BACA SENSOR
// =====================================================

AnalogSensorReading bacaPH() {
  int buffer_arr[10];
  int temp;

  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(PH_TANDON_PIN);
    delay(5);
  }

  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer_arr[i] > buffer_arr[j]) {
        temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }

  unsigned long avgval = 0;

  for (int i = 2; i < 8; i++) {
    avgval += buffer_arr[i];
  }

  float rataAdc = (float)avgval / 6.0;
  float volt = rataAdc * 3.3 / 4095.0;
  float ph = -5.70 * volt + calibration_value;

  return {rataAdc, volt, ph};
}

float bacaSuhu() {
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}

float bacaJarakUltrasonik(byte trigPin, byte echoPin, float suhuKompensasi) {
  const int jumlahSampel = 5;
  float totalJarak = 0;
  int sampelValid = 0;

  for (int i = 0; i < jumlahSampel; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long durasi = pulseIn(echoPin, HIGH, 30000UL);

    if (durasi > 0) {
      float cepatSuara = 331.3 + (0.606 * suhuKompensasi);
      float jarakCm = (durasi * cepatSuara) / 20000.0;

      totalJarak += jarakCm;
      sampelValid++;
    }

    delay(60);
  }

  if (sampelValid == 0) {
    return NAN;
  }

  return totalJarak / sampelValid;
}

float bacaJarakJsnTandon() {
  float sampel[JSN_TANDON_SAMPLE_COUNT];
  uint8_t jumlahValid = 0;

  for (uint8_t i = 0; i < JSN_TANDON_SAMPLE_COUNT; i++) {
    digitalWrite(JSN_TANDON_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(JSN_TANDON_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(JSN_TANDON_TRIG_PIN, LOW);

    unsigned long durasi = pulseIn(JSN_TANDON_ECHO_PIN, HIGH, 30000UL);

    if (durasi > 0) {
      float jarakCm = (durasi * 0.0343f) / 2.0f;

      if (jarakValid(jarakCm)) {
        sampel[jumlahValid++] = jarakCm;
      }
    }

    if (i + 1 < JSN_TANDON_SAMPLE_COUNT) {
      delay(JSN_TANDON_SAMPLE_GAP_MS);
    }
  }

  if (jumlahValid < JSN_TANDON_MIN_VALID_SAMPLES) {
    return NAN;
  }

  // Median dipakai agar pantulan sesaat seperti 60/90 cm tidak ikut menarik
  // hasil utama ketika mayoritas sampel tetap berada di sekitar nilai nyata.
  for (uint8_t i = 1; i < jumlahValid; i++) {
    float nilai = sampel[i];
    int8_t j = i - 1;

    while (j >= 0 && sampel[j] > nilai) {
      sampel[j + 1] = sampel[j];
      j--;
    }
    sampel[j + 1] = nilai;
  }

  if ((jumlahValid % 2) == 0) {
    return (sampel[(jumlahValid / 2) - 1] + sampel[jumlahValid / 2]) * 0.5f;
  }

  return sampel[jumlahValid / 2];
}

// =====================================================
// PERINTAH USER
// =====================================================

void bacaPerintahSerial() {
  String perintah = "";

  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (Serial.available()) {
      perintah = Serial.readStringUntil('\n');
    }

    xSemaphoreGive(serialMutex);
  }

  perintah.trim();
  perintah.toUpperCase();

  if (perintah.length() == 0) {
    return;
  }

  bool perintahAngka = true;
  for (uint8_t i = 0; i < perintah.length(); i++) {
    if (!isDigit(perintah.charAt(i))) {
      perintahAngka = false;
      break;
    }
  }

  if (perintahAngka) {
    float adcClearBaru = perintah.toFloat();

    if (adcClearBaru > 100 && adcClearBaru <= TURB_AWAL_ADC_MAX) {
      turbAwalAdcClear = adcClearBaru;
      saveTurbidityAdcClear();
      turbAwalEmaAdc = readTurbidityAwalAverage();
      turbTandonEmaAdc = readTurbidityTandonAverage();
      turbAwalEmaReady = true;
      turbTandonEmaReady = true;
      resetFilterKeputusanTurbidityAwal();

      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.print("[SET] ADC_CLEAR diperbarui: ");
        Serial.println((int)turbAwalAdcClear);
        xSemaphoreGive(serialMutex);
      }
    } else {
      serialPrintlnSafe("[!] Nilai tidak valid (harus 100-4095)");
    }

    return;
  }

  if (perintah.startsWith("TA=")) {
    float adcClearBaru = perintah.substring(3).toFloat();

    if (turbidityAdcClearValid(adcClearBaru)) {
      turbTandonAdcClear = adcClearBaru;
      saveTurbidityAdcClear();
      turbTandonEmaAdc = readTurbidityTandonAverage();
      turbTandonEmaReady = true;

      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.print("[TA] ADC_CLEAR tandon diperbarui: ");
        Serial.println((int)turbTandonAdcClear);
        xSemaphoreGive(serialMutex);
      }
    } else {
      serialPrintlnSafe("[!] Nilai tandon tidak valid (harus 100-4095)");
    }

    return;
  }

  if (perintah == "I") {
    printTurbidityAwalInfo();
    return;
  }

  if (perintah == "C") {
    float adcClearBaru = readTurbidityAwalAverage();

    if (adcClearBaru > 100 && adcClearBaru <= TURB_AWAL_ADC_MAX) {
      turbAwalAdcClear = adcClearBaru;
      saveTurbidityAdcClear();
      turbAwalEmaAdc = adcClearBaru;
      turbTandonEmaAdc = readTurbidityTandonAverage();
      turbAwalEmaReady = true;
      turbTandonEmaReady = true;
      resetFilterKeputusanTurbidityAwal();

      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.print("[C] ADC_CLEAR auto diset dari air bersih: ");
        Serial.println((int)turbAwalAdcClear);
        Serial.println("[C] Nilai bersih sekarang menjadi 0 NTU");
        xSemaphoreGive(serialMutex);
      }
    } else {
      serialPrintlnSafe("[!] Auto clear gagal, ADC tidak valid");
    }

    return;
  }

  if (perintah == "CT") {
    float adcClearBaru = readTurbidityTandonAverage();

    if (turbidityAdcClearValid(adcClearBaru)) {
      turbTandonAdcClear = adcClearBaru;
      saveTurbidityAdcClear();
      turbTandonEmaAdc = adcClearBaru;
      turbTandonEmaReady = true;

      if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.print("[CT] ADC_CLEAR tandon auto diset dari air bersih: ");
        Serial.println((int)turbTandonAdcClear);
        Serial.println("[CT] Nilai bersih tandon sekarang menjadi 0 NTU");
        xSemaphoreGive(serialMutex);
      }
    } else {
      serialPrintlnSafe("[!] Auto clear tandon gagal, ADC tidak valid");
    }

    return;
  }

  if (perintah == "T") {
    turbAwalShowRaw = !turbAwalShowRaw;
    serialPrintlnSafe(turbAwalShowRaw ? "[T] Raw ADC: ON" : "[T] Raw ADC: OFF");
    return;
  }

  if (MODE_KALIBRASI_SENSOR && (perintah == "KURAS_ON" || perintah == "KURAS_OFF")) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      setKurasTandonManual(false, "Serial command");
      xSemaphoreGive(dataMutex);
    }

    serialPrintlnSafe("Mode kalibrasi: perintah kuras diabaikan, relay tetap OFF");
    return;
  }

  if (perintah == "KURAS_ON") {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      setKurasTandonManual(true, "Serial command");
      xSemaphoreGive(dataMutex);
    }
  } else if (perintah == "KURAS_OFF") {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      setKurasTandonManual(false, "Serial command");
      xSemaphoreGive(dataMutex);
    }
  } else if (MODE_KALIBRASI_SENSOR) {
    serialPrintlnSafe("[?] Perintah: angka/C=awal, TA=4000/CT=tandon, I=info, T=raw");
  }
}

// =====================================================
// KONTROL PENAMPUNGAN AWAL
// =====================================================

void resetSiklusSolenoidKeruh() {
  solenoidKeruhPulseAktif = true;
  solenoidKeruhPulseLastMs = millis();
}

bool statusSiklusSolenoidKeruh() {
  const uint32_t now = millis();
  const uint32_t durasiSaatIni = solenoidKeruhPulseAktif ?
                                 SOLENOID_KERUH_AKTIF_MS :
                                 SOLENOID_KERUH_MATI_MS;

  if (now - solenoidKeruhPulseLastMs >= durasiSaatIni) {
    solenoidKeruhPulseAktif = !solenoidKeruhPulseAktif;
    solenoidKeruhPulseLastMs = now;
  }

  return solenoidKeruhPulseAktif;
}

void paksaRelayOffUntukKalibrasi() {
  digitalWrite(RELAY_PUMP_AWAL_12V_PIN, RELAY_OFF);
  digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_OFF);
  digitalWrite(RELAY_SOLENOID_KERUH_PIN, RELAY_OFF);
  digitalWrite(RELAY_SOLENOID_BERSIH_PIN, RELAY_OFF);

  pumpAwal12vAktif = false;
  pumpKuras5vAktif = false;
  solenoidKeruhAktif = false;
  solenoidBersihAktif = false;
  jalurAwalKeruhLatch = false;
  resetFilterKeputusanTurbidityAwal();
  resetSiklusSolenoidKeruh();
  triggerKurasTandon = false;
  perintahKurasTandon = false;
  statusSistem = STATUS_KALIBRASI;
}

void updateLatchMonitoringSensor() {
  if (sensorLevelAwalValid) {
    if (jarak_air_awal > BATAS_AIR_AWAL_MULAI_CM) {
      airAwalPenuhLatch = true;
    } else if (jarak_air_awal <= BATAS_AIR_AWAL_STOP_CM) {
      airAwalPenuhLatch = false;
    }
  } else {
    airAwalPenuhLatch = false;
  }

  if (sensorLevelTandonValid) {
    if (jarak_air_tandon <= BATAS_TANDON_BOLEH_KURAS_CM) {
      airTandonCukupLatch = true;
    } else if (jarak_air_tandon >= BATAS_TANDON_STOP_KURAS_CM) {
      airTandonCukupLatch = false;
    }
  } else {
    airTandonCukupLatch = false;
  }
}

void selesaikanPerintahKurasTandon(const char *pesan) {
  perintahKurasTandon = false;
  triggerKurasTandon = false;
  firebaseKurasCommandClearPending = true;
  serialPrintlnSafe(pesan);
}

void resetFilterKeputusanTurbidityAwal() {
  turbAwalKeruhConfirmCount = 0;
  turbAwalBersihConfirmCount = 0;
  turbAwalDecisionSeq = turbAwalReadingSeq;
  jalurAwalKeputusanSiap = false;
}

bool startupPompaAwalSedangDiabaikan() {
  return pumpAwal12vAktif &&
         (millis() - pumpAwalLastOnMs < TURB_AWAL_PUMP_STARTUP_IGNORE_MS);
}

bool keputusanJalurAwalKeruhStabil() {
  if (turbAwalDecisionSeq == turbAwalReadingSeq ||
      startupPompaAwalSedangDiabaikan()) {
    return jalurAwalKeruhLatch;
  }

  turbAwalDecisionSeq = turbAwalReadingSeq;

  if (jalurAwalKeruhLatch) {
    // Hysteresis: setelah masuk jalur keruh, baru kembali ke jalur bersih
    // jika nilai benar-benar turun sampai 20 NTU atau kurang.
    if (kekeruhan_awal <= BATAS_KERUH_AWAL_KEMBALI_BERSIH) {
      if (turbAwalBersihConfirmCount < TURB_AWAL_BERSIH_CONFIRM_READS) {
        turbAwalBersihConfirmCount++;
      }
    } else {
      turbAwalBersihConfirmCount = 0;
    }

    turbAwalKeruhConfirmCount = 0;

    if (turbAwalBersihConfirmCount >= TURB_AWAL_BERSIH_CONFIRM_READS) {
      jalurAwalKeruhLatch = false;
      jalurAwalKeputusanSiap = true;
      turbAwalBersihConfirmCount = 0;
    }

    return jalurAwalKeruhLatch;
  }

  if (kekeruhan_awal > BATAS_KERUH_AWAL) {
    if (turbAwalKeruhConfirmCount < TURB_AWAL_KERUH_CONFIRM_READS) {
      turbAwalKeruhConfirmCount++;
    }

    turbAwalBersihConfirmCount = 0;
  } else {
    // Saat memulai siklus, seluruh nilai <=25 NTU dikonfirmasi sebagai bersih.
    if (turbAwalBersihConfirmCount < TURB_AWAL_BERSIH_CONFIRM_READS) {
      turbAwalBersihConfirmCount++;
    }

    turbAwalKeruhConfirmCount = 0;
  }

  if (turbAwalKeruhConfirmCount >= TURB_AWAL_KERUH_CONFIRM_READS) {
    jalurAwalKeruhLatch = true;
    jalurAwalKeputusanSiap = true;
    turbAwalKeruhConfirmCount = 0;
  } else if (turbAwalBersihConfirmCount >= TURB_AWAL_BERSIH_CONFIRM_READS) {
    jalurAwalKeruhLatch = false;
    jalurAwalKeputusanSiap = true;
    turbAwalBersihConfirmCount = 0;
  }

  return jalurAwalKeruhLatch;
}

void markActuatorStatusChanged() {
  actuatorStatusDirty = true;
  actuatorStatusGeneration++;
}

void setInitialPump(bool aktif) {
  // Pompa hanya boleh ON jika route yang diminta sudah benar-benar terbuka.
  if (aktif && !solenoidRouteApplied(requestedSolenoidRoute)) {
    aktif = false;
  }

  bool sebelumnyaAktif = relayOutputAktif(RELAY_PUMP_AWAL_12V_PIN);

  if (sebelumnyaAktif == aktif) {
    pumpAwal12vAktif = sebelumnyaAktif;
    return;
  }

  digitalWrite(RELAY_PUMP_AWAL_12V_PIN, aktif ? RELAY_ON : RELAY_OFF);
  pumpAwal12vAktif = relayOutputAktif(RELAY_PUMP_AWAL_12V_PIN);
  markActuatorStatusChanged();

  if (pumpAwal12vAktif) {
    pumpAwalLastOnMs = millis();
  } else {
    relayBaruOff = true;
  }
}

void setCleanSolenoid(bool aktif) {
  if (aktif && solenoidKeruhAktif) {
    setTurbidSolenoid(false);
  }

  bool sebelumnyaAktif = relayOutputAktif(RELAY_SOLENOID_BERSIH_PIN);

  if (sebelumnyaAktif == aktif) {
    solenoidBersihAktif = sebelumnyaAktif;
    return;
  }

  digitalWrite(RELAY_SOLENOID_BERSIH_PIN, aktif ? RELAY_ON : RELAY_OFF);
  solenoidBersihAktif = relayOutputAktif(RELAY_SOLENOID_BERSIH_PIN);
  markActuatorStatusChanged();
}

void setTurbidSolenoid(bool aktif) {
  if (aktif && solenoidBersihAktif) {
    setCleanSolenoid(false);
  }

  bool sebelumnyaAktif = relayOutputAktif(RELAY_SOLENOID_KERUH_PIN);

  if (sebelumnyaAktif == aktif) {
    solenoidKeruhAktif = sebelumnyaAktif;
    return;
  }

  digitalWrite(RELAY_SOLENOID_KERUH_PIN, aktif ? RELAY_ON : RELAY_OFF);
  solenoidKeruhAktif = relayOutputAktif(RELAY_SOLENOID_KERUH_PIN);
  markActuatorStatusChanged();
}

bool solenoidRouteApplied(SolenoidRoute route) {
  if (solenoidTransitionState != SOLENOID_TRANSITION_IDLE) {
    return false;
  }

  if (route == SOLENOID_ROUTE_CLEAN) {
    return solenoidBersihAktif && !solenoidKeruhAktif;
  }

  if (route == SOLENOID_ROUTE_TURBID) {
    return solenoidKeruhAktif && !solenoidBersihAktif;
  }

  return !solenoidBersihAktif && !solenoidKeruhAktif;
}

void requestSolenoidRoute(SolenoidRoute route) {
  if (route != SOLENOID_ROUTE_NONE &&
      route != SOLENOID_ROUTE_CLEAN &&
      route != SOLENOID_ROUTE_TURBID) {
    route = SOLENOID_ROUTE_NONE;
  }

  if (route == requestedSolenoidRoute &&
      (solenoidTransitionState == SOLENOID_TRANSITION_DEADTIME ||
       solenoidRouteApplied(route))) {
    return;
  }

  // Urutan wajib saat berpindah: pompa OFF, kedua solenoid OFF, dead-time,
  // kemudian solenoid tujuan ON. Tidak ada delay yang memblokir task lain.
  setInitialPump(false);
  setCleanSolenoid(false);
  setTurbidSolenoid(false);
  requestedSolenoidRoute = route;

  if (route == SOLENOID_ROUTE_NONE) {
    solenoidTransitionState = SOLENOID_TRANSITION_IDLE;
    return;
  }

  solenoidTransitionStartedMs = millis();
  solenoidTransitionState = SOLENOID_TRANSITION_DEADTIME;
}

void processSolenoidTransition() {
  if (solenoidKeruhAktif && solenoidBersihAktif) {
    setInitialPump(false);
    setCleanSolenoid(false);
    setTurbidSolenoid(false);
    requestedSolenoidRoute = SOLENOID_ROUTE_NONE;
    solenoidTransitionState = SOLENOID_TRANSITION_IDLE;
    statusSistem = STATUS_SAFE_STOP;
    serialPrintlnSafe("SAFE STOP: kedua solenoid terdeteksi ON bersamaan");
    return;
  }

  if (solenoidTransitionState != SOLENOID_TRANSITION_DEADTIME ||
      millis() - solenoidTransitionStartedMs < SOLENOID_SWITCH_DEADTIME_MS) {
    return;
  }

  setInitialPump(false);

  if (requestedSolenoidRoute == SOLENOID_ROUTE_CLEAN) {
    setTurbidSolenoid(false);
    setCleanSolenoid(true);
  } else if (requestedSolenoidRoute == SOLENOID_ROUTE_TURBID) {
    setCleanSolenoid(false);
    setTurbidSolenoid(true);
  } else {
    setCleanSolenoid(false);
    setTurbidSolenoid(false);
  }

  solenoidTransitionState = SOLENOID_TRANSITION_IDLE;
}

void matikanSemuaAktuator(const String &alasan) {
  setInitialPump(false);
  digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_OFF);
  requestSolenoidRoute(SOLENOID_ROUTE_NONE);

  pumpKuras5vAktif = false;
  jalurAwalKeruhLatch = false;
  resetFilterKeputusanTurbidityAwal();
  resetSiklusSolenoidKeruh();
  relayBaruOff = true;
  statusSistem = STATUS_SAFE_STOP;

  String pesan("SAFE STOP: ");
  pesan.concat(alasan);
  serialPrintlnSafe(pesan);
}

void matikanAliranAwal() {
  setInitialPump(false);
  requestSolenoidRoute(SOLENOID_ROUTE_NONE);

  jalurAwalKeruhLatch = false;
  resetFilterKeputusanTurbidityAwal();
  resetSiklusSolenoidKeruh();
  relayBaruOff = true;
  statusSistem = STATUS_NORMAL;

  serialPrintlnSafe("ALIRAN AWAL OFF");
}

void runAutomaticFlowControl() {
  if (!sensorLevelAwalValid) {
    airAwalPenuhLatch = false;

    if (pumpAwal12vAktif || solenoidKeruhAktif || solenoidBersihAktif ||
        requestedSolenoidRoute != SOLENOID_ROUTE_NONE ||
        solenoidTransitionState != SOLENOID_TRANSITION_IDLE) {
      matikanSemuaAktuator("sensor level awal error saat aliran awal aktif");
    }

    return;
  }

  if (airAwalPenuhLatch) {
    if (jarak_air_awal <= BATAS_AIR_AWAL_STOP_CM) {
      airAwalPenuhLatch = false;
    }
  } else if (jarak_air_awal >= BATAS_AIR_AWAL_MULAI_CM) {
    airAwalPenuhLatch = true;
  }

  if (!airAwalPenuhLatch) {
    if (pumpAwal12vAktif || solenoidKeruhAktif || solenoidBersihAktif ||
        requestedSolenoidRoute != SOLENOID_ROUTE_NONE ||
        solenoidTransitionState != SOLENOID_TRANSITION_IDLE) {
      matikanAliranAwal();
    }

    return;
  }

  bool keputusanSebelumnyaSiap = jalurAwalKeputusanSiap;
  bool jalurSebelumnyaKeruh = jalurAwalKeruhLatch;
  bool jalurKeruh = keputusanJalurAwalKeruhStabil();

  // Jangan menghidupkan pompa sebelum klasifikasi turbidity stabil.
  // Ini mencegah air masuk ke jalur yang salah akibat satu pembacaan noise.
  if (!jalurAwalKeputusanSiap) {
    bool aliranSebelumnyaAktif = pumpAwal12vAktif ||
                                 solenoidKeruhAktif ||
                                 solenoidBersihAktif ||
                                 requestedSolenoidRoute != SOLENOID_ROUTE_NONE;

    if (aliranSebelumnyaAktif) {
      setInitialPump(false);
      requestSolenoidRoute(SOLENOID_ROUTE_NONE);
      serialPrintlnSafe("ALIRAN AWAL MENUNGGU KONFIRMASI NTU");
    }

    statusSistem = STATUS_NORMAL;
    return;
  }

  bool keputusanBaruSiap = !keputusanSebelumnyaSiap;
  bool jalurBerubah = keputusanSebelumnyaSiap &&
                      jalurSebelumnyaKeruh != jalurKeruh;

  if (jalurBerubah) {
    resetSiklusSolenoidKeruh();
  }

  SolenoidRoute targetRoute = jalurKeruh ?
      SOLENOID_ROUTE_TURBID : SOLENOID_ROUTE_CLEAN;
  requestSolenoidRoute(targetRoute);
  processSolenoidTransition();

  if (solenoidRouteApplied(targetRoute) && !pumpAwal12vAktif) {
    setInitialPump(true);
    statusSistem = STATUS_ALIR_AWAL;

    serialPrintlnSafe("PUMP 12V AWAL ON");
  }

  statusSistem = STATUS_ALIR_AWAL;

  if (keputusanBaruSiap || jalurBerubah) {
    serialPrintlnSafe(jalurKeruh ? "Jalur awal: SOLENOID KERUH ON, BERSIH OFF" :
                                   "Jalur awal: SOLENOID BERSIH ON, KERUH OFF");
  }
}

void runManualFlowControl() {
  // Type mismatch, field kosong, atau route di luar bersih/keruh selalu
  // menghasilkan kondisi aman. Command turbidity tidak dipakai di mode ini.
  if (!commandPumpAwal12vValid || !commandSolenoidRouteValid) {
    setInitialPump(false);
    requestSolenoidRoute(SOLENOID_ROUTE_NONE);
    statusSistem = STATUS_SAFE_STOP;
    return;
  }

  requestSolenoidRoute(commandSolenoidRoute);
  processSolenoidTransition();

  // Pakai command terakhir yang valid; Firebase putus sesaat tidak boleh
  // menahan pompa saat jalur solenoid sudah benar-benar terbuka.
  bool bolehMenyalakanPump = commandPumpAwal12v &&
                             solenoidRouteApplied(commandSolenoidRoute);
  setInitialPump(bolehMenyalakanPump);

  if (pumpAwal12vAktif) {
    statusSistem = STATUS_ALIR_AWAL;
  }
}

void kontrolPenampunganAwal() {
  if (!commandModeManualValid) {
    setInitialPump(false);
    requestSolenoidRoute(SOLENOID_ROUTE_NONE);
    statusSistem = STATUS_SAFE_STOP;
    return;
  }

  if (!controlModeInitialized || appliedModeManual != commandModeManual) {
    setInitialPump(false);
    requestSolenoidRoute(SOLENOID_ROUTE_NONE);
    resetFilterKeputusanTurbidityAwal();
    jalurAwalKeruhLatch = false;
    airAwalPenuhLatch = false;
    appliedModeManual = commandModeManual;
    controlModeInitialized = true;
    serialPrintlnSafe(appliedModeManual ?
                      "Mode aliran: MANUAL" : "Mode aliran: OTOMATIS");
  }

  if (appliedModeManual) {
    runManualFlowControl();
  } else {
    runAutomaticFlowControl();
  }
}

// =====================================================
// KONTROL TANDON AKHIR
// =====================================================

void kontrolKurasTandon() {
  if (sensorLevelTandonValid) {
    if (jarak_air_tandon <= BATAS_TANDON_BOLEH_KURAS_CM) {
      airTandonCukupLatch = true;
    } else if (jarak_air_tandon >= BATAS_TANDON_STOP_KURAS_CM) {
      airTandonCukupLatch = false;
    }
  } else {
    airTandonCukupLatch = false;
  }

  if (perintahKurasTandon && !pumpKuras5vAktif) {
    digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_ON);

    pumpKuras5vAktif = true;
    triggerKurasTandon = true;
    statusSistem = STATUS_KURAS_TANDON;

    serialPrintlnSafe("PUMP 5V KURAS TANDON ON");
  }

  if (pumpKuras5vAktif && !perintahKurasTandon) {
    digitalWrite(RELAY_PUMP_KURAS_5V_PIN, RELAY_OFF);

    pumpKuras5vAktif = false;
    triggerKurasTandon = false;
    relayBaruOff = true;
    statusSistem = STATUS_NORMAL;

    serialPrintlnSafe("PUMP 5V KURAS TANDON OFF - USER");
  }

  if (!perintahKurasTandon) {
    triggerKurasTandon = false;
  }
}

void updateStatusSistem() {
  if (pumpAwal12vAktif) {
    statusSistem = STATUS_ALIR_AWAL;
    return;
  }

  if (pumpKuras5vAktif) {
    statusSistem = STATUS_KURAS_TANDON;
    return;
  }

  if (!commandModeManualValid ||
      (appliedModeManual &&
       (!commandPumpAwal12vValid || !commandSolenoidRouteValid))) {
    statusSistem = STATUS_SAFE_STOP;
    return;
  }

  if (!sensorLevelAwalValid || !sensorLevelTandonValid || !sensorSuhuValid || !sensorPhValid) {
    statusSistem = STATUS_SENSOR_ERROR;
    return;
  }

  statusSistem = STATUS_NORMAL;
}

// =====================================================
// SERIAL
// =====================================================

void tampilTurbidityRawCsv() {
  if (millis() - turbidityRawLastMs < TURB_AWAL_READ_MS) {
    return;
  }

  turbidityRawLastMs = millis();

  TurbidityRawReading reading = bacaTurbidityAwalRaw();
  float ntuAwal = 0.0;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    ntuAwal = reading.ntu;
    kekeruhan_awal_raw = reading.adcRaw;
    kekeruhan_awal = ntuAwal;
    adc_turb_awal = reading.adcEma;
    volt_turb_awal_raw = reading.voltRaw;
    volt_turb_awal = reading.voltAvg;
    turbAwalReadingSeq++;
    xSemaphoreGive(dataMutex);
  }

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    const char *statusAir = ntuAwal <= TURB_AWAL_NTU_THRESHOLD ? "JERNIH" : "KERUH !";

    Serial.print("NTU: ");
    if (ntuAwal < 10) {
      Serial.print("  ");
    } else if (ntuAwal < 100) {
      Serial.print(" ");
    }
    Serial.print(ntuAwal, 1);
    Serial.print("  |  ");
    Serial.print(statusAir);

    if (turbAwalShowRaw) {
      Serial.print("  |  ADC: ");
      Serial.print((int)reading.adcEma);
    }

    Serial.println();
    xSemaphoreGive(serialMutex);
  }
}

void tampilSerial() {
  if (MODE_KALIBRASI_SENSOR) {
    tampilTurbidityRawCsv();
    return;
  }

  float kekeruhanAwalSnapshot = 0;
  float kekeruhanAwalRawSnapshot = 0;
  float jarakAwalSnapshot = 0;
  float jarakAwalRawSnapshot = 0;
  float adcTurbAwalSnapshot = 0;
  float voltTurbAwalSnapshot = 0;
  float kekeruhanTandonSnapshot = 0;
  float kekeruhanTandonRawSnapshot = 0;
  float phTandonSnapshot = 0;
  float suhuTandonSnapshot = 0;
  float jarakTandonSnapshot = 0;
  float adcTurbTandonSnapshot = 0;
  float voltTurbTandonSnapshot = 0;
  float voltTurbTandonRawSnapshot = 0;
  float adcPhTandonSnapshot = 0;
  float voltPhTandonSnapshot = 0;
  bool pumpAwalSnapshot = false;
  bool pumpKurasSnapshot = false;
  bool solenoidKeruhSnapshot = false;
  bool solenoidBersihSnapshot = false;
  bool perintahKurasSnapshot = false;
  bool levelAwalValidSnapshot = false;
  bool levelTandonValidSnapshot = false;
  bool suhuValidSnapshot = false;
  bool phValidSnapshot = false;
  bool airAwalPenuhSnapshot = false;
  bool airTandonCukupSnapshot = false;
  StatusSistem statusSnapshot = STATUS_INIT;

  if (xSemaphoreTake(dataMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  if (millis() - lastSerialUpdate < SERIAL_INTERVAL) {
    xSemaphoreGive(dataMutex);
    return;
  }

  lastSerialUpdate = millis();

  kekeruhanAwalSnapshot = kekeruhan_awal;
  kekeruhanAwalRawSnapshot = kekeruhan_awal_raw;
  jarakAwalSnapshot = jarak_air_awal;
  jarakAwalRawSnapshot = jarak_air_awal_raw;
  adcTurbAwalSnapshot = adc_turb_awal;
  voltTurbAwalSnapshot = volt_turb_awal;
  kekeruhanTandonSnapshot = kekeruhan_tandon;
  kekeruhanTandonRawSnapshot = kekeruhan_tandon_raw;
  phTandonSnapshot = ph_tandon;
  suhuTandonSnapshot = suhu_tandon;
  jarakTandonSnapshot = jarak_air_tandon;
  adcTurbTandonSnapshot = adc_turb_tandon;
  voltTurbTandonSnapshot = volt_turb_tandon;
  voltTurbTandonRawSnapshot = volt_turb_tandon_raw;
  adcPhTandonSnapshot = adc_ph_tandon;
  voltPhTandonSnapshot = volt_ph_tandon;
  pumpAwalSnapshot = pumpAwal12vAktif;
  pumpKurasSnapshot = pumpKuras5vAktif;
  solenoidKeruhSnapshot = solenoidKeruhAktif;
  solenoidBersihSnapshot = solenoidBersihAktif;
  perintahKurasSnapshot = perintahKurasTandon;
  levelAwalValidSnapshot = sensorLevelAwalValid;
  levelTandonValidSnapshot = sensorLevelTandonValid;
  suhuValidSnapshot = sensorSuhuValid;
  phValidSnapshot = sensorPhValid;
  airAwalPenuhSnapshot = airAwalPenuhLatch;
  airTandonCukupSnapshot = airTandonCukupLatch;
  statusSnapshot = statusSistem;

  xSemaphoreGive(dataMutex);

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {

    Serial.println("================================");
    Serial.print("Status sistem : ");
    Serial.println(statusSistemText(statusSnapshot));
    Serial.print("Mode kalibrasi : ");
    Serial.println(MODE_KALIBRASI_SENSOR ? "AKTIF - RELAY OFF" : "NONAKTIF");

    Serial.println("PENAMPUNGAN AWAL");

    Serial.print("Turbidity awal NTU : ");
    Serial.println(kekeruhanAwalSnapshot, 1);
    Serial.print("Turbidity awal ADC raw : ");
    Serial.println(kekeruhanAwalRawSnapshot, 1);
    Serial.print("Turbidity awal ADC EMA/Volt avg : ");
    Serial.print(adcTurbAwalSnapshot, 1);
    Serial.print(" / ");
    Serial.println(voltTurbAwalSnapshot, 3);

    Serial.print("Tinggi air awal HC-SR04 : ");
    Serial.println(levelAwalValidSnapshot ? formatJarak(jarakAwalSnapshot) : "ERROR");
    Serial.print("Jarak sensor awal raw/offset : ");
    Serial.print(levelAwalValidSnapshot ? formatJarak(jarakAwalRawSnapshot) : "ERROR");
    Serial.print(" / +");
    Serial.println(KALIBRASI_LEVEL_AWAL_OFFSET_CM, 1);

    Serial.print("Air awal penuh : ");
    Serial.println(airAwalPenuhSnapshot ? "YA" : "TIDAK");

    Serial.print("Pump 12V awal : ");
    Serial.println(pumpAwalSnapshot ? "ON" : "OFF");

    Serial.print("Solenoid 1 keruh : ");
    Serial.println(solenoidKeruhSnapshot ? "ON" : "OFF");

    Serial.print("Solenoid 2 bersih : ");
    Serial.println(solenoidBersihSnapshot ? "ON" : "OFF");

    Serial.println("TANDON AKHIR");

    Serial.print("pH tandon : ");
    if (phValidSnapshot) {
      Serial.println(phTandonSnapshot, 2);
    } else {
      Serial.println("ERROR");
    }
    Serial.print("pH tandon ADC/Volt : ");
    Serial.print(adcPhTandonSnapshot, 1);
    Serial.print(" / ");
    Serial.println(voltPhTandonSnapshot, 3);

    Serial.print("Turbidity tandon : ");
    Serial.println(kekeruhanTandonSnapshot, 1);
    Serial.print("Turbidity tandon ADC raw : ");
    Serial.println(kekeruhanTandonRawSnapshot, 1);
    Serial.print("Turbidity tandon ADC/Volt : ");
    Serial.print(adcTurbTandonSnapshot, 1);
    Serial.print(" / ");
    Serial.println(voltTurbTandonSnapshot, 3);
    Serial.print("Turbidity tandon Volt raw : ");
    Serial.println(voltTurbTandonRawSnapshot, 3);

    Serial.print("Suhu tandon : ");
    if (suhuValidSnapshot) {
      Serial.println(suhuTandonSnapshot, 1);
    } else {
      Serial.println("ERROR");
    }

    Serial.print("Level tandon JSN-SR04T : ");
    Serial.println(levelTandonValidSnapshot ? formatJarak(jarakTandonSnapshot) : "ERROR");

    Serial.print("Air tandon cukup kuras : ");
    Serial.println(airTandonCukupSnapshot ? "YA" : "TIDAK");

    Serial.print("Perintah kuras user : ");
    Serial.println(perintahKurasSnapshot ? "ON" : "OFF");

    Serial.print("Pump 5V kuras tandon : ");
    Serial.println(pumpKurasSnapshot ? "ON" : "OFF");

    Serial.println("================================");

    xSemaphoreGive(serialMutex);
  }
}

// =====================================================
// FREERTOS TASK
// =====================================================

void taskBacaSensor(void *parameter) {
  registerTaskWatchdog();
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    bool bacaSensorNonLevel = false;
    float suhuKompensasi = 25.0f;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      bacaSensorNonLevel = !sensorNonLevelPaused &&
                           !pumpAwal12vAktif &&
                           !pumpKuras5vAktif;

      if (sensorSuhuValid) {
        suhuKompensasi = suhu_tandon;
      }

      xSemaphoreGive(dataMutex);
    }

    TurbidityRawReading turbAwalBaru{};
    TurbidityRawReading turbTandonBaru{};
    AnalogSensorReading phTandonBaru{};
    float suhuTandonBaru = NAN;

    if (bacaSensorNonLevel) {
      turbAwalBaru = bacaTurbidityAwalRaw();
      watchdogResetSafe();
      turbTandonBaru = bacaTurbidityTandonRaw();
      watchdogResetSafe();
      phTandonBaru = bacaPH();
      watchdogResetSafe();
      suhuTandonBaru = bacaSuhu();
      watchdogResetSafe();

      if (suhuValid(suhuTandonBaru)) {
        suhuKompensasi = suhuTandonBaru;
      }
    }

    // Kedua sensor level selalu dibaca, termasuk ketika salah satu pompa ON.
    float jarakAwalBaru = bacaJarakUltrasonik(HC_AWAL_TRIG_PIN,
                                               HC_AWAL_ECHO_PIN,
                                               suhuKompensasi);
    watchdogResetSafe();
    float jarakAwalTerkalibrasi = jarakAwalBaru + KALIBRASI_LEVEL_AWAL_OFFSET_CM;
    float tinggiAwalBaru = hitungTinggiAirAwal(jarakAwalTerkalibrasi);

    float jarakTandonBaru = bacaJarakJsnTandon();
    watchdogResetSafe();

    bool sensorLevelAwalValidBaru = jarakValid(jarakAwalBaru);
    bool sensorSuhuValidBaru = bacaSensorNonLevel && suhuValid(suhuTandonBaru);
    bool sensorPhValidBaru = bacaSensorNonLevel && phValid(phTandonBaru.nilai);

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      bool simpanSensorNonLevel = bacaSensorNonLevel &&
                                  !sensorNonLevelPaused &&
                                  !pumpAwal12vAktif &&
                                  !pumpKuras5vAktif;

      if (simpanSensorNonLevel) {
        bool smoothResume = sensorNonLevelSmoothReadsRemaining > 0;
        bool adaPembacaanTurbiditySebelumnya = turbAwalReadingSeq > 0;
        float alpha = smoothResume ? SENSOR_NON_LEVEL_RESUME_ALPHA :
                                     FILTER_SENSOR_ALPHA;

        kekeruhan_awal_raw = turbAwalBaru.adcRaw;
        kekeruhan_awal = smoothResume ?
            filterSensorDenganAlpha(kekeruhan_awal,
                                    turbAwalBaru.ntu,
                                    adaPembacaanTurbiditySebelumnya,
                                    true,
                                    alpha) :
            turbAwalBaru.ntu;
        adc_turb_awal = turbAwalBaru.adcEma;
        volt_turb_awal_raw = turbAwalBaru.voltRaw;
        volt_turb_awal = turbAwalBaru.voltAvg;
        kekeruhan_tandon_raw = turbTandonBaru.adcRaw;
        kekeruhan_tandon = smoothResume ?
            filterSensorDenganAlpha(kekeruhan_tandon,
                                    turbTandonBaru.ntu,
                                    adaPembacaanTurbiditySebelumnya,
                                    true,
                                    alpha) :
            turbTandonBaru.ntu;
        adc_turb_tandon = turbTandonBaru.adcEma;
        volt_turb_tandon_raw = turbTandonBaru.voltRaw;
        volt_turb_tandon = turbTandonBaru.voltAvg;
        turbAwalReadingSeq++;
        ph_tandon = filterSensorDenganAlpha(ph_tandon,
                                            phTandonBaru.nilai,
                                            sensorPhValid,
                                            sensorPhValidBaru,
                                            alpha);
        adc_ph_tandon = filterSensorDenganAlpha(adc_ph_tandon,
                                                phTandonBaru.adc,
                                                true,
                                                true,
                                                alpha);
        volt_ph_tandon = filterSensorDenganAlpha(volt_ph_tandon,
                                                 phTandonBaru.volt,
                                                 true,
                                                 true,
                                                 alpha);
        suhu_tandon = filterSensorDenganAlpha(suhu_tandon,
                                              suhuTandonBaru,
                                              sensorSuhuValid,
                                              sensorSuhuValidBaru,
                                              alpha);
        sensorSuhuValid = sensorSuhuValidBaru;
        sensorPhValid = sensorPhValidBaru;

        if (smoothResume) {
          sensorNonLevelSmoothReadsRemaining--;
        }
      }

      jarak_air_awal_raw = filterSensor(jarak_air_awal_raw, jarakAwalBaru,
                                        sensorLevelAwalValid,
                                        sensorLevelAwalValidBaru);
      jarak_air_awal = filterSensor(jarak_air_awal, tinggiAwalBaru,
                                    sensorLevelAwalValid,
                                    sensorLevelAwalValidBaru);
      prosesPembacaanJsnTandon(jarakTandonBaru);

      sensorLevelAwalValid = sensorLevelAwalValidBaru;
      xSemaphoreGive(dataMutex);
    }

    watchdogResetSafe();
    vTaskDelayUntil(&lastWakeTime, SENSOR_TASK_INTERVAL);
  }
}

void taskKontrolSistem(void *parameter) {
  registerTaskWatchdog();
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      if (MODE_KALIBRASI_SENSOR) {
        paksaRelayOffUntukKalibrasi();
        updateLatchMonitoringSensor();
      } else {
        kontrolPenampunganAwal();
        kontrolKurasTandon();
        updateStatusSistem();
      }

      updatePauseSensorNonLevel();

      xSemaphoreGive(dataMutex);
    }

    watchdogResetSafe();
    vTaskDelayUntil(&lastWakeTime, CONTROL_TASK_INTERVAL);
  }
}

void taskLcd(void *parameter) {
  registerTaskWatchdog();
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    updateLCD();
    watchdogResetSafe();
    vTaskDelayUntil(&lastWakeTime, LCD_TASK_INTERVAL);
  }
}

void taskSerialCommand(void *parameter) {
  registerTaskWatchdog();
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    bacaPerintahSerial();
    watchdogResetSafe();
    vTaskDelayUntil(&lastWakeTime, SERIAL_COMMAND_TASK_INTERVAL);
  }
}

void taskSerialLog(void *parameter) {
  registerTaskWatchdog();
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    tampilSerial();
    watchdogResetSafe();
    vTaskDelayUntil(&lastWakeTime, SERIAL_LOG_TASK_INTERVAL);
  }
}

void taskWiFi(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    bool wifiTerhubung = WiFi.status() == WL_CONNECTED;
    uint32_t now = millis();
    bool reconnectBaruDimulai = false;
    bool reconnectSelesai = false;
    uint32_t durasiReconnect = 0;
    uint32_t urutanReconnect = 0;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      if (!wifiTerhubung && wifiPernahTerhubung &&
          !reconnectSedangBerlangsung) {
        reconnectStart = now;
        reconnectSedangBerlangsung = true;
        reconnectBaruDimulai = true;
      }

      if (wifiTerhubung) {
        wifiPernahTerhubung = true;

        if (reconnectSedangBerlangsung) {
          reconnectEnd = now;
          reconnectTime = reconnectEnd - reconnectStart;
          totalReconnectTime += reconnectTime;
          reconnectCount++;
          reconnectSedangBerlangsung = false;
          reconnectSelesai = true;
          durasiReconnect = reconnectTime;
          urutanReconnect = reconnectCount;
        }
      }

      xSemaphoreGive(dataMutex);
    }

    if (!wifiTerhubung &&
        (reconnectBaruDimulai || lastWiFiReconnectAttempt == 0 ||
         now - lastWiFiReconnectAttempt >= WIFI_RECONNECT_RETRY_MS)) {
      lastWiFiReconnectAttempt = now;
      serialPrintlnSafe("WiFi terputus, mencoba reconnect...");
      WiFi.reconnect();
    }

    if (reconnectSelesai) {
      String pesan("WiFi terhubung kembali. Reconnect ke-");
      pesan.concat(urutanReconnect);
      pesan.concat(", waktu: ");
      pesan.concat(durasiReconnect);
      pesan.concat(" ms");
      serialPrintlnSafe(pesan);
    }

    vTaskDelayUntil(&lastWakeTime, WIFI_CHECK_INTERVAL);
  }
}

void taskFirebase(void *parameter) {
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    TickType_t now = xTaskGetTickCount();
    bool firebaseOnline = WiFi.status() == WL_CONNECTED && Firebase.ready();
    bool waktunyaRealtime = now - lastFirebaseRealtimeSend >= FIREBASE_REALTIME_INTERVAL;
    bool waktunyaHistory = FIREBASE_HISTORY_AKTIF &&
                           now - lastFirebaseHistoryPush >= FIREBASE_HISTORY_INTERVAL;
    bool waktunyaAppsScript = !appsScriptPernahDikirim ||
                              now - lastAppsScriptSend >= APPS_SCRIPT_INTERVAL;

    if (WiFi.status() == WL_CONNECTED && waktunyaAppsScript) {
      FirebasePayload payload = ambilFirebasePayloadSnapshot();

      if (kirimSensorKeSpreadsheet(payload)) {
        lastAppsScriptSend = now;
        appsScriptPernahDikirim = true;
      }
    }

    if (firebaseOnline) {
      if (!firebaseReadyLogged) {
        firebaseReadyLogged = true;
        serialPrintlnSafe("Firebase siap.");
      }

      bool resetPumpPending = resetInitialPumpCommandAfterConnect();

      if (!MODE_KALIBRASI_SENSOR) {
        bool resetCommandPending = resetPerintahKurasFirebaseJikaPerlu();

        if (!resetPumpPending && !resetCommandPending) {
          readControlCommands();
        }
      }

      if (!waktunyaRealtime) {
        publishActuatorStatus();
      }

      if (waktunyaRealtime) {
        FirebasePayload payload = ambilFirebasePayloadSnapshot();
        sendRealtimePayloadToFirebase(payload);
        lastFirebaseRealtimeSend = xTaskGetTickCount();
      }

      if (waktunyaHistory) {
        FirebasePayload payload = ambilFirebasePayloadSnapshot();
        pushHistoryPayloadToFirebase(payload);
      }

      if (now - lastFirebaseQueueFlush >= FIREBASE_QUEUE_FLUSH_INTERVAL) {
        flushPendingFirebaseQueue();
        lastFirebaseQueueFlush = now;
      }
    } else {
      firebaseReadyLogged = false;
      setFirebaseControlConnected(false);

      if (waktunyaHistory) {
        FirebasePayload payload = ambilFirebasePayloadSnapshot();
        simpanHistoryPayloadLokal(payload);
      }
    }

    vTaskDelayUntil(&lastWakeTime, FIREBASE_TASK_INTERVAL);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  // Harus menjadi aksi pertama saat firmware mulai. Semua aktuator tetap OFF
  // selama LCD, sensor, Wi-Fi, dan Firebase diinisialisasi.
  inisialisasiRelayAman();

  Serial.begin(115200);
  Serial.setTimeout(20);

  Wire.begin();

  lcd.init();
  lcd.backlight();

  sensors.begin();

  pinMode(HC_AWAL_TRIG_PIN, OUTPUT);
  pinMode(HC_AWAL_ECHO_PIN, INPUT);
  pinMode(JSN_TANDON_TRIG_PIN, OUTPUT);
  pinMode(JSN_TANDON_ECHO_PIN, INPUT);

  digitalWrite(HC_AWAL_TRIG_PIN, LOW);
  digitalWrite(JSN_TANDON_TRIG_PIN, LOW);

  if (MODE_KALIBRASI_SENSOR) {
    statusSistem = STATUS_KALIBRASI;
  }

  analogReadResolution(TURB_AWAL_ADC_BITS);
  analogSetPinAttenuation(TURB_AWAL_PIN, ADC_11db);
  analogSetPinAttenuation(TURB_TANDON_PIN, ADC_11db);
  analogSetPinAttenuation(PH_TANDON_PIN, ADC_11db);

  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  esp_task_wdt_init(WATCHDOG_TIMEOUT_SECONDS, true);

  if (dataMutex == NULL || serialMutex == NULL) {
    lcd.clear();
    printLcdLine(0, "RTOS ERROR");
    printLcdLine(1, "MUTEX FAILED");

    while (true) {
      delay(1000);
    }
  }

  EEPROM.begin(TURB_AWAL_EEPROM_SIZE);
  loadTurbidityAdcClear();
  turbAwalEmaAdc = readTurbidityAwalAverage();
  turbTandonEmaAdc = readTurbidityTandonAverage();
  turbAwalEmaReady = true;
  turbTandonEmaReady = true;

  printTurbidityAwalBanner();
  printTurbidityAwalInfo();

  lcd.clear();
  printLcdLine(0, "SISTEM AIR");
  printLcdLine(1, "INITIALIZING");
  delay(2000);

  initFirebase();

  lcd.clear();
  tampilSensorLCD(lcdPage,
                  kekeruhan_awal,
                  jarak_air_awal,
                  kekeruhan_tandon,
                  ph_tandon,
                  suhu_tandon,
                  jarak_air_tandon,
                  pumpAwal12vAktif,
                  pumpKuras5vAktif,
                  solenoidKeruhAktif,
                  solenoidBersihAktif,
                  perintahKurasTandon,
                  statusSistem,
                  sensorLevelAwalValid,
                  sensorLevelTandonValid);

  bool taskOk = true;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskBacaSensor,
                                             "BacaSensor",
                                             6144,
                                             NULL,
                                             1,
                                             NULL,
                                             0) == pdPASS;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskKontrolSistem,
                                             "Kontrol",
                                             4096,
                                             NULL,
                                             3,
                                             NULL,
                                             1) == pdPASS;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskSerialCommand,
                                             "SerialCmd",
                                             3072,
                                             NULL,
                                             2,
                                             NULL,
                                             1) == pdPASS;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskLcd,
                                             "LCD",
                                             4096,
                                             NULL,
                                             1,
                                             NULL,
                                             1) == pdPASS;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskSerialLog,
                                             "SerialLog",
                                             4096,
                                             NULL,
                                             1,
                                             NULL,
                                             1) == pdPASS;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskWiFi,
                                             "WiFi",
                                             4096,
                                             NULL,
                                             1,
                                             NULL,
                                             0) == pdPASS;

  taskOk = taskOk && xTaskCreatePinnedToCore(taskFirebase,
                                             "Firebase",
                                             16384,
                                             NULL,
                                             1,
                                             NULL,
                                             1) == pdPASS;

  if (!taskOk) {
    matikanSemuaAktuator("task FreeRTOS gagal dibuat");
    lcd.clear();
    printLcdLine(0, "RTOS ERROR");
    printLcdLine(1, "TASK FAILED");

    while (true) {
      delay(1000);
    }
  }
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
