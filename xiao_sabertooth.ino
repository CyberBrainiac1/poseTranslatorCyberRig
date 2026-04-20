#include <Arduino.h>
#include <SoftwareSerial.h>

#define SABERTOOTH_TX_PIN D6
#define SABERTOOTH_RX_PIN D7
#define SABERTOOTH_BAUD_RATE 9600
#define WATCHDOG_TIMEOUT_MS 500

static const uint8_t MOTOR1_STOP_BYTE = 64;
static const uint8_t MOTOR2_STOP_BYTE = 192;

SoftwareSerial sabertoothSerial(SABERTOOTH_RX_PIN, SABERTOOTH_TX_PIN);

unsigned long lastValidPacketMs = 0;
unsigned long lastBlinkMs = 0;
bool ledState = false;
bool watchdogTriggered = false;
bool hasReceivedPacket = false;
uint8_t inputBuffer[2] = {MOTOR1_STOP_BYTE, MOTOR2_STOP_BYTE};
uint8_t inputIndex = 0;

uint8_t clampMotor1Byte(uint8_t value) {
  if (value < 64) {
    return 64;
  }
  if (value > 127) {
    return 127;
  }
  return value;
}

uint8_t clampMotor2Byte(uint8_t value) {
  if (value < 192) {
    return 192;
  }
  if (value > 255) {
    return 255;
  }
  return value;
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
  const uint8_t safeMotor1 = clampMotor1Byte(motor1Byte);
  const uint8_t safeMotor2 = clampMotor2Byte(motor2Byte);
  sabertoothSerial.write(safeMotor1);
  sabertoothSerial.write(safeMotor2);
}

void sendStopBytes() {
  sendSabertoothBytes(MOTOR1_STOP_BYTE, MOTOR2_STOP_BYTE);
}

void handleLedState() {
  if (watchdogTriggered) {
    blinkLed(100);
    return;
  }

  if (hasReceivedPacket && millis() - lastValidPacketMs <= WATCHDOG_TIMEOUT_MS) {
    ledState = true;
    setLed(true);
    return;
  }

  blinkLed(1000);
}

void processUsbSerial() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) {
      continue;
    }

    inputBuffer[inputIndex] = static_cast<uint8_t>(incoming);
    inputIndex++;

    if (inputIndex >= 2) {
      inputIndex = 0;
      const uint8_t motor1Byte = clampMotor1Byte(inputBuffer[0]);
      const uint8_t motor2Byte = clampMotor2Byte(inputBuffer[1]);
      sendSabertoothBytes(motor1Byte, motor2Byte);
      lastValidPacketMs = millis();
      hasReceivedPacket = true;
      watchdogTriggered = false;
      ledState = true;
      setLed(true);
    }
  }
}

void checkWatchdog() {
  if (hasReceivedPacket && millis() - lastValidPacketMs > WATCHDOG_TIMEOUT_MS) {
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
  lastValidPacketMs = millis();
  lastBlinkMs = millis();
  ledState = false;
  setLed(false);
}

void loop() {
  processUsbSerial();
  checkWatchdog();
  handleLedState();
}
