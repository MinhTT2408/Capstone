// PPGModule.cpp - PPG sensor + BLE + SD card implementation
#include "PPGModule.h"
#include "config.h"
#include <Wire.h>
#include "MAX30105.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

namespace PPGModule {

// ===================== INTERNAL STATE =====================

// Sensor
static MAX30105 particleSensor;
static float currentPPG = 0;
static float dc = 0;
static bool dcInitialized = false;

// BLE
static BLECharacteristic *pPPGCharacteristic = nullptr;
static BLECharacteristic *pLevelCharacteristic = nullptr;
static BLECharacteristic *pHistoryCharacteristic = nullptr;
static bool deviceConnected = false;

// Compression level from app (1-4 active, 0 = stop)
static volatile int compressionLevel = 0;

// RTOS-safe flags (set from BLE callbacks, processed in update())
static volatile bool fetchHistoryRequested = false;
static volatile bool startSessionRequested = false;
static volatile bool endSessionRequested = false;

// Session state
static bool _isSessionActive = false;
static bool _sessionJustStarted = false;
static bool _sessionJustEnded = false;

// Analytics
static double sumPPG = 0;
static long countPPG = 0;
static unsigned long sessionStartTime = 0;

// Timing
static unsigned long lastBleTime = 0;
static unsigned long lastSdWriteTime = 0;

// SD Card
static bool sdAvailable = false;
static String currentFilename;

// Global stop flag for motor abort
volatile bool stopMotorRequested = false;

// ===================== BLE CALLBACKS =====================

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("[PPG] BLE Connected");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("[PPG] BLE Disconnected");
    
    // If session was active, request end (processed safely in update())
    if (_isSessionActive) {
      endSessionRequested = true;
    }
    // Restart advertising
    BLEDevice::startAdvertising();
  }
};

class LevelCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (!value.empty()) {
      int newLevel = value[0];
      Serial.printf("[PPG] Received level: %d\n", newLevel);

      if (newLevel == 99) {
        // Special command: fetch history
        fetchHistoryRequested = true;
      } 
      else if (newLevel == 0) {
        // Stop session
        endSessionRequested = true;
        compressionLevel = 0;
      } 
      else if (newLevel >= 1 && newLevel <= NUM_LEVELS) {
        // Start or update session
        if (!_isSessionActive) {
          startSessionRequested = true;
        }
        compressionLevel = newLevel;
      }
    }
  }
};

// ===================== SD CARD HELPERS =====================

static String xorCipher(const String &input) {
  String output = input;
  for (size_t i = 0; i < output.length(); i++) {
    if (output[i] != '\n' && output[i] != '\r') {
      output[i] = output[i] ^ ENCRYPTION_KEY;
    }
  }
  return output;
}

static String getNextFilename() {
  char filename[16];
  for (int i = 1; i <= 9999; i++) {
    snprintf(filename, sizeof(filename), "/%04d.csv", i);
    if (!SD.exists(filename)) return String(filename);
  }
  return "/overflow.csv";
}

static void logToSD(float ppgValue) {
  if (!sdAvailable || currentFilename.isEmpty()) return;

  unsigned long elapsed = millis() - sessionStartTime;
  String line = String(elapsed) + "," + String(ppgValue) + "," + String(compressionLevel);
  String encrypted = xorCipher(line);

  File f = SD.open(currentFilename, FILE_APPEND);
  if (f) {
    f.println(encrypted);
    f.close();
  }
}

// ===================== INTERNAL HELPERS =====================

static void startNewSession() {
  Serial.println("[PPG] --- NEW SESSION ---");
  sumPPG = 0;
  countPPG = 0;
  sessionStartTime = millis();
  _isSessionActive = true;
  _sessionJustStarted = true;
  stopMotorRequested = false;  // Clear any previous stop request

  // Create new session file on SD
  if (sdAvailable) {
    currentFilename = getNextFilename();
    File f = SD.open(currentFilename, FILE_WRITE);
    if (f) {
      // Write encrypted header
      String header = "TimeMs,PPG,Level";
      f.println(xorCipher(header));
      f.close();
      Serial.printf("[SD] Session file: %s\n", currentFilename.c_str());
    } else {
      Serial.println("[SD] Failed to create session file!");
    }
  }
  lastSdWriteTime = millis();
}

static void doEndSession() {
  if (!_isSessionActive) return;
  
  Serial.println("[PPG] --- ENDING SESSION ---");
  _isSessionActive = false;
  _sessionJustEnded = true;
  stopMotorRequested = true;  // Signal motor loops to abort
  
  // Calculate and log stats
  float avg = (countPPG > 0) ? (float)(sumPPG / countPPG) : 0;
  unsigned long duration = millis() - sessionStartTime;
  Serial.printf("[PPG] Session lasted %lu s, avg PPG: %.1f, %ld samples\n",
                duration / 1000, avg, countPPG);

  // Append summary to /history.txt on SD
  if (sdAvailable) {
    File f = SD.open("/history.txt", FILE_APPEND);
    if (f) {
      // Match original ppg.text history format expected by app
      String cleanName = currentFilename;
      cleanName.replace("/", ""); cleanName.replace(".csv", "");
      String summary = "ID:" + cleanName + ",Time:" + String(duration / 1000) + "s,Avg:" + String(avg, 1);
      f.println(xorCipher(summary));
      f.close();
      Serial.println("[SD] Summary appended to /history.txt");
    }
  }
  currentFilename = "";
  compressionLevel = 0;
}

static void processHistoryRequest() {
  if (!deviceConnected || pHistoryCharacteristic == nullptr) return;

  if (!sdAvailable || !SD.exists("/history.txt")) {
    Serial.println("[PPG] History requested — no data");
    String msg = "NO_HISTORY";
    pHistoryCharacteristic->setValue(msg.c_str());
    pHistoryCharacteristic->notify();
    return;
  }

  Serial.println("[PPG] Sending history over BLE...");
  File f = SD.open("/history.txt", FILE_READ);
  if (!f) return;

  while (f.available()) {
    String encLine = f.readStringUntil('\n');
    encLine.trim();
    if (encLine.isEmpty()) continue;

    // Decrypt and send over BLE
    String decrypted = xorCipher(encLine);
    pHistoryCharacteristic->setValue(decrypted.c_str());
    pHistoryCharacteristic->notify();
    Serial.println("Sent: " + decrypted);
    delay(50);  // Match original timing to prevent BLE buffer overflow
  }
  f.close();
  Serial.println("[PPG] --- End of History ---");
}

// ===================== PUBLIC API =====================

void begin() {
  // Initialize I2C for MAX30105 on dedicated pins
  Wire.begin(PPG_I2C_SDA, PPG_I2C_SCL);
  
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("[PPG] MAX30105 not found! Check wiring.");
  } else {
    // Configure sensor: LED brightness, sample avg, mode, sample rate, pulse width, ADC range
    particleSensor.setup(0x1F, 4, 2, 400, 411, 4096);
    Serial.println("[PPG] MAX30105 initialized");
  }

  // Initialize SD card (matches original ppg.text init)
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Card mount FAILED — logging disabled");
    sdAvailable = false;
  } else {
    sdAvailable = true;
    Serial.printf("[SD] Card mounted, size: %llu MB\n", SD.cardSize() / (1024 * 1024));
  }

  // Initialize BLE
  BLEDevice::init("ESP32_PPG_DVT");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  
  BLEService *pService = pServer->createService(PPG_SERVICE_UUID);

  // PPG data characteristic (notify to app)
  pPPGCharacteristic = pService->createCharacteristic(
    PPG_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pPPGCharacteristic->addDescriptor(new BLE2902());

  // Level characteristic (app writes compression level)
  pLevelCharacteristic = pService->createCharacteristic(
    LEVEL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  pLevelCharacteristic->setCallbacks(new LevelCallbacks());

  // History characteristic (notify to app)
  pHistoryCharacteristic = pService->createCharacteristic(
    HISTORY_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pHistoryCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(PPG_SERVICE_UUID);
  BLEDevice::getAdvertising()->start();

  Serial.println("[PPG] BLE advertising started");
}

void update() {
  unsigned long now = millis();
  
  // Reset one-shot flags from previous cycle
  _sessionJustStarted = false;
  _sessionJustEnded = false;

  // --- Process BLE request flags safely on main thread ---
  if (fetchHistoryRequested) {
    fetchHistoryRequested = false;
    processHistoryRequest();
  }
  if (startSessionRequested) {
    startSessionRequested = false;
    startNewSession();
  }
  if (endSessionRequested) {
    endSessionRequested = false;
    doEndSession();
  }

  // --- Read PPG sensor ---
  long irValue = particleSensor.getIR();
  if (!dcInitialized) { dc = irValue; dcInitialized = true; }
  dc = 0.95f * dc + 0.05f * irValue;
  float ac = irValue - dc;
  currentPPG = ac * 2.0f + 500.0f;
  currentPPG = constrain(currentPPG, 0.0f, 1000.0f);

  // --- BLE: send PPG at 50 Hz ---
  if (now - lastBleTime >= BLE_PPG_INTERVAL_MS) {
    lastBleTime = now;
    if (deviceConnected && pPPGCharacteristic != nullptr) {
      pPPGCharacteristic->setValue((uint8_t*)&currentPPG, sizeof(float));
      pPPGCharacteristic->notify();
    }
  }

  // --- Accumulate session analytics ---
  if (_isSessionActive) {
    sumPPG += currentPPG;
    countPPG++;

    // --- Log to SD card at fixed interval ---
    if (sdAvailable && (now - lastSdWriteTime >= SD_WRITE_INTERVAL_MS)) {
      lastSdWriteTime = now;
      logToSD(currentPPG);
    }
  }
}

bool isSessionActive() {
  return _isSessionActive;
}

bool sessionJustStarted() {
  bool val = _sessionJustStarted;
  // Don't clear here — cleared at start of next update()
  return val;
}

bool sessionJustEnded() {
  bool val = _sessionJustEnded;
  return val;
}

void endSession() {
  doEndSession();
}

int getCompressionLevel() {
  return compressionLevel;
}

bool isBleConnected() {
  return deviceConnected;
}

} // namespace PPGModule
