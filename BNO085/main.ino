// === MAVEN firmware — Arduino + NimBLE + BNO08x ===
// Streams Q,tms,qi,qj,qk,qr,ax,ay,az at PRINT_HZ over BLE NUS.
// Game rotation vector (6-axis, no mag) + raw accel (gravity in).

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_BNO08x.h>

// ----- USER CONFIG -----
static const char* DEVICE_NAME = "NameBLEIMU1";
static const int   IMU_ID      = 1;
static const int   PRINT_HZ    = 100;
static const long  REPORT_INTERVAL_US = 1000000L / 100;   // 100 Hz per report

// BNO08x: I2C, no hardware reset wire
#define BNO08X_RESET -1

// ----- UUIDs (must match dashboard) -----
static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

// ----- BLE state -----
NimBLECharacteristic* txChar = nullptr;
bool deviceConnected = false;
bool streamEnabled   = true;

// ----- IMU state (latched latest of each report) -----
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

volatile float qi=0, qj=0, qk=0, qr=1;
volatile float ax=0, ay=0, az=0;     // raw accel, m/s^2, gravity included
bool haveQ = false, haveA = false;

// ----- Rate meters -----
uint32_t imuFrames = 0, bleMsgs = 0;
uint32_t rateLast = 0;

// ----- Connection callbacks -----
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    deviceConnected = true;
    Serial.println("[BLE] central connected");
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    deviceConnected = false;
    Serial.printf("[BLE] disconnected, reason=%d\n", reason);
    NimBLEDevice::startAdvertising();
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
    String msg = String(pChar->getValue().c_str());
    msg.trim();
    if (msg.length() == 0) return;
    String up = msg; up.toUpperCase();
    Serial.print("[BLE] RX: "); Serial.println(msg);

    if (up == "ID?") {
      String r = "ID," + String(IMU_ID) + "\n";
      txChar->setValue((uint8_t*)r.c_str(), r.length()); txChar->notify();
    } else if (up.startsWith("STREAM,ON")) {
      streamEnabled = true;
      const char* r = "ACK STREAM ON\n";
      txChar->setValue((uint8_t*)r, strlen(r)); txChar->notify();
    } else if (up.startsWith("STREAM,OFF")) {
      streamEnabled = false;
      const char* r = "ACK STREAM OFF\n";
      txChar->setValue((uint8_t*)r, strlen(r)); txChar->notify();
    } else if (up.startsWith("CAL")) {
      const char* r = "ACK CAL STUB\n";
      txChar->setValue((uint8_t*)r, strlen(r)); txChar->notify();
    } else {
      String r = "ERR Unknown cmd: " + msg + "\n";
      txChar->setValue((uint8_t*)r.c_str(), r.length()); txChar->notify();
    }
  }
};

void enableReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US))
    Serial.println("[BNO] could not enable game RV");
  if (!bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US))
    Serial.println("[BNO] could not enable accelerometer");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== MAVEN boot ===");

  // BLE
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  NimBLEService* svc = server->createService(NUS_SERVICE_UUID);
  txChar = svc->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* rxChar = svc->createCharacteristic(
    NUS_RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rxChar->setCallbacks(new RxCB());
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setName(DEVICE_NAME);
  adv->enableScanResponse(true);
  NimBLEDevice::startAdvertising();
  Serial.printf("Advertising as %s\n", DEVICE_NAME);

  // BNO08x
  if (!bno08x.begin_I2C()) {
    Serial.println("[BNO] not found on I2C — halting");
    while (1) delay(10);
  }
  Serial.println("[BNO] found");
  enableReports();

  rateLast = millis();
}

void loop() {
  // Drain any sensor events that arrived since last loop.
  if (bno08x.wasReset()) {
    Serial.println("[BNO] sensor was reset — re-enabling reports");
    enableReports();
  }
  while (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR: {
        auto& v = sensorValue.un.gameRotationVector;
        qi = v.i; qj = v.j; qk = v.k; qr = v.real;
        haveQ = true;
        break;
      }
      case SH2_ACCELEROMETER: {
        auto& a = sensorValue.un.accelerometer;
        ax = a.x; ay = a.y; az = a.z;
        haveA = true;
        break;
      }
      default: break;
    }
    imuFrames++;
  }

  // Stream Q at PRINT_HZ once we have at least one of each report.
  static uint32_t lastTx = 0;
  const uint32_t TX_DT = 1000 / PRINT_HZ;
  uint32_t now = millis();

  if (deviceConnected && streamEnabled && haveQ && haveA
      && (now - lastTx >= TX_DT)) {
    lastTx = now;
    char buf[160];
    int n = snprintf(buf, sizeof(buf),
      "Q,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
      (unsigned long)now,
      qi, qj, qk, qr,
      ax, ay, az);
    txChar->setValue((uint8_t*)buf, n);
    txChar->notify();
    bleMsgs++;
  }

  // Rate report every 5 s
  if (now - rateLast >= 5000) {
    float dt = (now - rateLast) / 1000.0f;
    float imuHz = imuFrames / dt;     // includes both reports combined
    float bleHz = bleMsgs   / dt;
    Serial.printf("RATE IMU~%.1fHz (combined), BLE~%.1fHz\n", imuHz, bleHz);
    if (deviceConnected) {
      char r[80];
      int n = snprintf(r, sizeof(r),
        "RATE IMU~%.1fHz, BLE~%.1fHz, PRINT_HZ=%d\n", imuHz, bleHz, PRINT_HZ);
      txChar->setValue((uint8_t*)r, n);
      txChar->notify();
    }
    imuFrames = 0; bleMsgs = 0; rateLast = now;
  }
}
