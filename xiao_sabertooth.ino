#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#define SABERTOOTH_TX_PIN D6
#define SABERTOOTH_RX_PIN D7
#define SABERTOOTH_BAUD_RATE 9600
#define POT_LEFT_PIN A0
#define POT_RIGHT_PIN A1
#define WATCHDOG_TIMEOUT_MS 500
#define POT_LOSS_TIMEOUT_MS 200

static const uint8_t MOTOR1_STOP_BYTE = 64;
static const uint8_t MOTOR2_STOP_BYTE = 192;
static const uint8_t EEPROM_MAGIC = 0xAB;
static const float SOFT_STOP_ZONE = 0.05f;
static const float HARD_STOP_ZONE = 0.01f;

SoftwareSerial sabertoothSerial(SABERTOOTH_RX_PIN, SABERTOOTH_TX_PIN);

int pot_min_l = 0;
int pot_max_l = 4095;
int pot_min_r = 0;
int pot_max_r = 4095;

unsigned long last_command_time = 0;
unsigned long lastBlinkMs = 0;
unsigned long pot_loss_timer_l = 0;
unsigned long pot_loss_timer_r = 0;
bool pot_loss_l = false;
bool pot_loss_r = false;
bool watchdogTriggered = false;
bool hasReceivedPacket = false;
bool ledState = false;

uint8_t inputBuffer[2] = {MOTOR1_STOP_BYTE, MOTOR2_STOP_BYTE};
uint8_t inputIndex = 0;
bool calibrationMode = false;
char calibrationBuffer[96];
uint8_t calibrationIndex = 0;

void eepromBegin() {
#if defined(ARDUINO_ARCH_RP2040)
  EEPROM.begin(64);
#endif
}

void eepromCommit() {
#if defined(ARDUINO_ARCH_RP2040)
  EEPROM.commit();
#endif
}

int readInt16(int address) {
  const int lo = EEPROM.read(address);
  const int hi = EEPROM.read(address + 1);
  return (hi << 8) | lo;
}

void writeInt16(int address, int value) {
  EEPROM.write(address, value & 0xFF);
  EEPROM.write(address + 1, (value >> 8) & 0xFF);
}

void loadCalibration() {
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    pot_min_l = readInt16(1);
    pot_max_l = readInt16(3);
    pot_min_r = readInt16(5);
    pot_max_r = readInt16(7);
    if (pot_min_l < 0 || pot_min_l > 4095 || pot_max_l < 0 || pot_max_l > 4095 || pot_min_l >= pot_max_l) {
      pot_min_l = 0;
      pot_max_l = 4095;
    }
    if (pot_min_r < 0 || pot_min_r > 4095 || pot_max_r < 0 || pot_max_r > 4095 || pot_min_r >= pot_max_r) {
      pot_min_r = 0;
      pot_max_r = 4095;
    }
  }
}

void saveCalibration(int lMin, int lMax, int rMin, int rMax) {
  EEPROM.write(0, EEPROM_MAGIC);
  writeInt16(1, lMin);
  writeInt16(3, lMax);
  writeInt16(5, rMin);
  writeInt16(7, rMax);
  eepromCommit();
  pot_min_l = lMin;
  pot_max_l = lMax;
  pot_min_r = rMin;
  pot_max_r = rMax;
}

uint8_t clampMotor1Byte(int value) {
  if (value < 64) {
    return 64;
  }
  if (value > 127) {
    return 127;
  }
  return static_cast<uint8_t>(value);
}

uint8_t clampMotor2Byte(int value) {
  if (value < 192) {
    return 192;
  }
  if (value > 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

float normalizedPosition(int adc, int minAdc, int maxAdc) {
  if (maxAdc <= minAdc) {
    return 0.0f;
  }
  float pos = (static_cast<float>(adc) - static_cast<float>(minAdc)) / static_cast<float>(maxAdc - minAdc);
  if (pos < 0.0f) {
    return 0.0f;
  }
  if (pos > 1.0f) {
    return 1.0f;
  }
  return pos;
}

float stopScale(float pos) {
  if (pos < HARD_STOP_ZONE || pos > (1.0f - HARD_STOP_ZONE)) {
    return 0.0f;
  }
  if (pos < SOFT_STOP_ZONE) {
    return pos / SOFT_STOP_ZONE;
  }
  if (pos > (1.0f - SOFT_STOP_ZONE)) {
    return (1.0f - pos) / SOFT_STOP_ZONE;
  }
  return 1.0f;
}

uint8_t scaleMotor1(uint8_t motorByte, int adc) {
  float command = (static_cast<float>(clampMotor1Byte(motorByte)) - 64.0f) / 63.0f * 127.0f;
  command *= stopScale(normalizedPosition(adc, pot_min_l, pot_max_l));
  int mapped = static_cast<int>((command / 127.0f) * 63.0f + 64.0f);
  return clampMotor1Byte(mapped);
}

uint8_t scaleMotor2(uint8_t motorByte, int adc) {
  float command = (static_cast<float>(clampMotor2Byte(motorByte)) - 192.0f) / 63.0f * 127.0f;
  command *= stopScale(normalizedPosition(adc, pot_min_r, pot_max_r));
  int mapped = static_cast<int>((command / 127.0f) * 63.0f + 192.0f);
  return clampMotor2Byte(mapped);
}

void setLed(bool on) {
  digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

void blinkLed(unsigned long periodMs) {
  const unsigned long now = millis();
  const unsigned long halfPeriod = periodMs / 2;
  if (now - lastBlinkMs >= halfPeriod) {
    lastBlinkMs = now;
    ledState = !ledState;
    setLed(ledState);
  }
}

void sendSabertoothBytes(uint8_t motor1Byte, uint8_t motor2Byte) {
  sabertoothSerial.write(clampMotor1Byte(motor1Byte));
  sabertoothSerial.write(clampMotor2Byte(motor2Byte));
}

void sendStopBytes() {
  sendSabertoothBytes(MOTOR1_STOP_BYTE, MOTOR2_STOP_BYTE);
}

bool parseCalibrationValue(const char *line, const char *key, int &value) {
  const char *found = strstr(line, key);
  if (found == nullptr) {
    return false;
  }
  found += strlen(key);
  value = atoi(found);
  return value >= 0 && value <= 4095;
}

void handleCalibrationLine(const char *line) {
  int lMin = 0;
  int lMax = 0;
  int rMin = 0;
  int rMax = 0;
  if (!parseCalibrationValue(line, "L_MIN:", lMin) ||
      !parseCalibrationValue(line, "L_MAX:", lMax) ||
      !parseCalibrationValue(line, "R_MIN:", rMin) ||
      !parseCalibrationValue(line, "R_MAX:", rMax) ||
      lMin >= lMax ||
      rMin >= rMax) {
    Serial.print("CAL_ERR\n");
    return;
  }
  saveCalibration(lMin, lMax, rMin, rMax);
  Serial.print("CAL_OK\n");
}

void feedCalibrationChar(char incoming) {
  if (calibrationIndex < sizeof(calibrationBuffer) - 1) {
    calibrationBuffer[calibrationIndex++] = incoming;
  }
  if (incoming == '\n' || incoming == '\r') {
    calibrationBuffer[calibrationIndex] = '\0';
    handleCalibrationLine(calibrationBuffer);
    calibrationMode = false;
    calibrationIndex = 0;
  }
}

void updatePotLoss(bool atLimit, unsigned long &timer, bool &lossFlag, const char *errText) {
  const unsigned long now = millis();
  if (atLimit) {
    if (timer == 0) {
      timer = now;
    }
    if (!lossFlag && now - timer > POT_LOSS_TIMEOUT_MS) {
      lossFlag = true;
      Serial.print(errText);
      sendStopBytes();
    }
  } else {
    timer = 0;
  }
}

void handleLedState() {
  if (pot_loss_l || pot_loss_r || watchdogTriggered) {
    blinkLed(100);
    return;
  }
  if (hasReceivedPacket && millis() - last_command_time <= WATCHDOG_TIMEOUT_MS) {
    ledState = true;
    setLed(true);
    return;
  }
  blinkLed(1000);
}

void processUsbSerial(int adc_l, int adc_r) {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      continue;
    }

    if (calibrationMode) {
      feedCalibrationChar(static_cast<char>(incoming));
      continue;
    }

    if (incoming == 'C') {
      calibrationMode = true;
      calibrationIndex = 0;
      feedCalibrationChar('C');
      continue;
    }

    inputBuffer[inputIndex] = static_cast<uint8_t>(incoming);
    inputIndex++;
    if (inputIndex >= 2) {
      inputIndex = 0;
      if (pot_loss_l || pot_loss_r) {
        pot_loss_l = false;
        pot_loss_r = false;
        pot_loss_timer_l = 0;
        pot_loss_timer_r = 0;
      }
      uint8_t motor1Byte = clampMotor1Byte(inputBuffer[0]);
      uint8_t motor2Byte = clampMotor2Byte(inputBuffer[1]);
      motor1Byte = scaleMotor1(motor1Byte, adc_l);
      motor2Byte = scaleMotor2(motor2Byte, adc_r);
      sendSabertoothBytes(motor1Byte, motor2Byte);
      last_command_time = millis();
      hasReceivedPacket = true;
      watchdogTriggered = false;
    }
  }
}

void checkWatchdog() {
  if (hasReceivedPacket && millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    if (!watchdogTriggered) {
      sendStopBytes();
      watchdogTriggered = true;
      lastBlinkMs = millis();
    }
  }
}

void setup() {
  sabertoothSerial.begin(SABERTOOTH_BAUD_RATE);
  sendStopBytes();
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  analogReadResolution(12);
  eepromBegin();
  loadCalibration();
  last_command_time = millis();
  lastBlinkMs = millis();
  setLed(false);
}

void loop() {
  const int adc_l = analogRead(POT_LEFT_PIN);
  const int adc_r = analogRead(POT_RIGHT_PIN);

  Serial.print("L=");
  Serial.print(adc_l);
  Serial.print(",R=");
  Serial.print(adc_r);
  Serial.print("\n");

  updatePotLoss(adc_l == 0 || adc_l == 4095, pot_loss_timer_l, pot_loss_l, "ERR=POT_L\n");
  updatePotLoss(adc_r == 0 || adc_r == 4095, pot_loss_timer_r, pot_loss_r, "ERR=POT_R\n");
  if (pot_loss_l || pot_loss_r) {
    sendStopBytes();
    handleLedState();
    processUsbSerial(adc_l, adc_r);
    return;
  }

  processUsbSerial(adc_l, adc_r);
  checkWatchdog();
  handleLedState();
}
