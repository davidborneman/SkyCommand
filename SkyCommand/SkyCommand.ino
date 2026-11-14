//#define __DEBUG__
//#define __WARN__

#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Pin Definitions (Verified: White TX = Pin 4, Green RX = Pin 2)
const int txPin = 4;
const int rxPin = 2;

// Timing Constants (Verified Skywalker V1 Protocol)
const int one_length = 1100;              // Threshold for interpreting a bit as '1' (us)
const unsigned long FRAME_GAP_US = 10000; // 10 ms clean HIGH idle gap between TX frames

// Default Startup / Safe-Idle Actuator Values (0 - 100%)
const uint8_t DEFAULT_VENT   = 0;   // OT2: Exhaust Fan off
const uint8_t DEFAULT_FILTER = 0;   // Filter Fan off
const uint8_t DEFAULT_COOL   = 0;   // Cooling Tray Fan off
const uint8_t DEFAULT_DRUM   = 0;   // Drum Motor off
const uint8_t DEFAULT_HEAT   = 0;   // OT1: FIR Heater off
const char    DEFAULT_UNITS  = 'F'; // Default temperature unit ('F' or 'C')

// Buffer Sizes
const int roasterLength = 7;    // Bytes
const int controllerLength = 6; // Bytes

// Buffer Definitions
uint8_t receiveBuffer[roasterLength] = {0};
uint8_t sendBuffer[controllerLength] = {0};

// Interrupt RX Capture Buffers (Pin 2 / INT1)
volatile uint8_t isrBuffer[roasterLength] = {0};
volatile uint8_t isrBitIndex = 0;
volatile bool isrSync = false;
volatile bool rxFrameReady = false;
volatile unsigned long rxFallMicros = 0;

// Byte Indices
const int ventByte   = 0;
const int filterByte = 1;
const int coolByte   = 2;
const int drumByte   = 3;
const int heatByte   = 4;
const int checkByte  = 5;

double temp = 0.0;
double tempC = 0.0;
char CorF = DEFAULT_UNITS;

// Link & Failsafe State
const int maxTemp = 300; // in Celsius
unsigned long lastEventTime = 0;
const unsigned long lastEventTimeout = 10000000UL; // 10 seconds in microseconds
unsigned long lastFrameEndMicros = 0;
unsigned long offLockoutUntil = 0;
bool roasterLinkActive = false;

// Non-blocking Hardware Interrupt (INT1 on Digital Pin 2)
void rxPinISR() {
  unsigned long now = micros();
  if (digitalRead(rxPin) == LOW) {
    rxFallMicros = now;
    return;
  }

  // Do not overwrite an unread completed frame while main loop copies it
  if (rxFrameReady) return;

  unsigned long lowWidth = now - rxFallMicros;

  // 1. ~7.5 ms LOW preamble marks the start of a 7-byte (56-bit) telemetry frame
  if (lowWidth > 5000 && lowWidth < 10000) {
    isrBitIndex = 0;
    isrSync = true;
    for (uint8_t i = 0; i < roasterLength; i++) {
      isrBuffer[i] = 0;
    }
  }
  // 2. Decode 56 data bits (LSB-first: ~1.5 ms LOW = 1, ~600 us LOW = 0)
  else if (isrSync && isrBitIndex < (roasterLength * 8) && lowWidth >= 300 && lowWidth <= 2500) {
    if (lowWidth > one_length) {
      isrBuffer[isrBitIndex / 8] |= (1 << (isrBitIndex % 8));
    }
    isrBitIndex++;

    if (isrBitIndex == (roasterLength * 8)) {
      isrSync = false;
      rxFrameReady = true;
    }
  }
}

void setControlChecksum() {
  uint8_t sum = 0;
  for (int i = 0; i < checkByte; i++) {
    sum += sendBuffer[i];
  }
  sendBuffer[checkByte] = sum;
}

void setValue(uint8_t* bytePtr, uint8_t v) {
  *bytePtr = v;
  setControlChecksum();
}

void applyDefaults() {
  sendBuffer[ventByte]   = DEFAULT_VENT;
  sendBuffer[filterByte] = DEFAULT_FILTER;
  sendBuffer[coolByte]   = DEFAULT_COOL;
  sendBuffer[drumByte]   = DEFAULT_DRUM;
  sendBuffer[heatByte]   = DEFAULT_HEAT;
  setControlChecksum();
}

void pulsePin(int pin, int duration) {
  digitalWrite(pin, LOW);
  delayMicroseconds(duration);
  digitalWrite(pin, HIGH);
}

void sendRoasterMessage() {
  // Send Preamble (~7.5 ms LOW)
  pulsePin(txPin, 7500);
  delayMicroseconds(3800);

  // Send 6-byte Message (LSB-first: 1500 us = 1, 600 us = 0, 750 us HIGH gap)
  for (int i = 0; i < controllerLength; i++) {
    for (int j = 0; j < 8; j++) {
      if (bitRead(sendBuffer[i], j) == 1) {
        pulsePin(txPin, 1500); // logic 1
      } else {
        pulsePin(txPin, 600);  // logic 0
      }
      delayMicroseconds(750);  // inter-bit gap
    }
  }

  lastFrameEndMicros = micros();
}

void enableRoasterLink() {
  if (!roasterLinkActive) {
    noInterrupts();
    isrBitIndex = 0;
    isrSync = false;
    rxFrameReady = false;
    interrupts();

    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, HIGH);
    delay(25); // Clean 25 ms HIGH idle level before first preamble

    pinMode(rxPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(rxPin), rxPinISR, CHANGE);

    lastEventTime = micros();
    lastFrameEndMicros = micros() - FRAME_GAP_US; // Transmit immediately on current loop
    roasterLinkActive = true;
  }
}

void disableRoasterLink() {
  if (roasterLinkActive) {
    // Flush 3 zeroed frames spaced 10 ms apart so actuators halt cleanly before sleep
    applyDefaults();
    for (uint8_t k = 0; k < 3; k++) {
      sendRoasterMessage();
      delay(10);
    }

    detachInterrupt(digitalPinToInterrupt(rxPin));

    // Park TX actively HIGH (idle mark state) so noise cannot trigger false start pulses
    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, HIGH);
    pinMode(rxPin, INPUT);

    roasterLinkActive = false;
  }
}

void shutdown() {
  applyDefaults();
}

void eStop() {
  setValue(&sendBuffer[heatByte], 0);
  setValue(&sendBuffer[ventByte], 100);
}

double calculateTemp() {
  double x = (((uint16_t)receiveBuffer[0] << 8) | receiveBuffer[1]) / 1000.0;
  double y = (((uint16_t)receiveBuffer[2] << 8) | receiveBuffer[3]) / 1000.0;

  double v = 583.1509258523457 + -714.0345395202813 * x + -196.071718077524 * y
             + 413.37964344228334 * x * x + 2238.149675349052 * x * y
             + -4099.91031297056 * y * y + 357.49007607425233 * x * x * x
             + -5001.419602972793 * x * x * y + 8242.08618555862 * x * y * y
             + 247.6124684730026 * y * y * y + -555.8643213534281 * x * x * x * x
             + 3879.431274654493 * x * x * x * y + -6885.682277959339 * x * x * y * y
             + 2868.4191998911865 * x * y * y * y + -1349.1588373011923 * y * y * y * y;

  double c = (v - 32.0) * 5.0 / 9.0;
  if (isnan(c) || c < -20.0 || c > 400.0) {
    return temp; // Reject out-of-bounds polynomial artifact
  }

  tempC = c;
  return (CorF == 'C') ? tempC : v;
}

bool calculateRoasterChecksum() {
  uint8_t sum = 0;
  for (int i = 0; i < (roasterLength - 1); i++) {
    sum += receiveBuffer[i];
  }
  return sum == receiveBuffer[roasterLength - 1];
}

void processRoasterFrame() {
  if (!rxFrameReady) return;

  noInterrupts();
  for (uint8_t i = 0; i < roasterLength; i++) {
    receiveBuffer[i] = isrBuffer[i];
  }
  rxFrameReady = false;
  interrupts();

  if (calculateRoasterChecksum()) {
    temp = calculateTemp();
  }
}

uint8_t clampPercent(uint8_t value) {
  return (value > 100) ? 100 : value;
}

void handleHEAT(uint8_t value) {
  setValue(&sendBuffer[heatByte], clampPercent(value));
  lastEventTime = micros();
}

void handleVENT(uint8_t value) {
  setValue(&sendBuffer[ventByte], clampPercent(value));
  lastEventTime = micros();
}

void handleCOOL(uint8_t value) {
  setValue(&sendBuffer[coolByte], clampPercent(value));
  lastEventTime = micros();
}

void handleFILTER(uint8_t value) {
  setValue(&sendBuffer[filterByte], clampPercent(value));
  lastEventTime = micros();
}

void handleDRUM(uint8_t value) {
  setValue(&sendBuffer[drumByte], (value != 0) ? 100 : 0);
  lastEventTime = micros();
}

void handleCHAN() {
  Serial.println("# Active channels set to 0200");
  lastEventTime = micros();
}

void handleREAD() {
  Serial.print(0.0);
  Serial.print(',');
  Serial.print(temp);
  Serial.print(',');
  Serial.print(temp);
  Serial.print(',');
  Serial.print(sendBuffer[heatByte]);
  Serial.print(',');
  Serial.print(sendBuffer[ventByte]);
  Serial.print(',');
  Serial.println('0');

  lastEventTime = micros();
}

void getArtisanMessage() {
  if (Serial.available() <= 0) return;

  char cmdBuf[32];
  size_t len = Serial.readBytesUntil('\n', cmdBuf, sizeof(cmdBuf) - 1);
  cmdBuf[len] = '\0';

  // Trim trailing \r, spaces, or tabs
  while (len > 0 && (cmdBuf[len - 1] == '\r' || cmdBuf[len - 1] == ' ' || cmdBuf[len - 1] == '\t')) {
    cmdBuf[--len] = '\0';
  }
  if (len == 0) return;

  uint8_t value = 0;
  char* paramStr = NULL;
  char* split = strchr(cmdBuf, ';');
  if (split != NULL) {
    *split = '\0';
    paramStr = split + 1;
    long parsed = strtol(paramStr, NULL, 10);
    if (parsed < 0) parsed = 0;
    if (parsed > 255) parsed = 255;
    value = (uint8_t)parsed;
  }

  if (strcmp(cmdBuf, "OFF") == 0) {
    offLockoutUntil = micros() + 1500000UL; // Ignore stray trailing READ for 1.5 s
    shutdown();
    disableRoasterLink();
    return;
  }

  bool wakeLink = true;

  if (strcmp(cmdBuf, "READ") == 0) {
    handleREAD();
    // Prevent a trailing READ right after OFF from re-enabling the roaster link
    if ((long)(micros() - offLockoutUntil) < 0) {
      wakeLink = false;
    }
  } else if (strcmp(cmdBuf, "OT1") == 0) {
    handleHEAT(value);
  } else if (strcmp(cmdBuf, "OT2") == 0) {
    handleVENT(value);
  } else if (strcmp(cmdBuf, "ESTOP") == 0) {
    eStop();
    wakeLink = false;
  } else if (strcmp(cmdBuf, "DRUM") == 0) {
    handleDRUM(value);
  } else if (strcmp(cmdBuf, "FILTER") == 0) {
    handleFILTER(value);
  } else if (strcmp(cmdBuf, "COOL") == 0) {
    handleCOOL(value);
  } else if (strcmp(cmdBuf, "CHAN") == 0) {
    handleCHAN();
  } else if (strcmp(cmdBuf, "UNITS") == 0) {
    if (paramStr != NULL && (paramStr[0] == 'C' || paramStr[0] == 'F')) {
      CorF = paramStr[0];
    }
    lastEventTime = micros();
  } else {
    wakeLink = false;
  }

  if (wakeLink) {
    enableRoasterLink();
  }
}

bool itsbeentoolong() {
  return ((micros() - lastEventTime) > lastEventTimeout);
}

bool isTemperatureOverLimit() {
  return (!isnan(tempC) && tempC > maxTemp);
}

void failSafeChecks() {
  if (roasterLinkActive && itsbeentoolong()) {
    shutdown();
    disableRoasterLink();
  }
  if (isTemperatureOverLimit()) {
    eStop();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(15);

  applyDefaults();

  // Park TX actively HIGH (idle mark state) at boot so roaster stays cleanly asleep until Artisan ON
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);
  pinMode(rxPin, INPUT);

  lastEventTime = micros();
}

void loop() {
  failSafeChecks();

  // Transmit continuously at 10 ms inter-frame spacing while Artisan is ON
  if (roasterLinkActive && (micros() - lastFrameEndMicros >= FRAME_GAP_US)) {
    sendRoasterMessage();
  }

  // Decode 56-bit telemetry frames captured via INT1
  processRoasterFrame();

  // Handle Artisan serial commands
  if (Serial.available() > 0) {
    getArtisanMessage();
  }
}