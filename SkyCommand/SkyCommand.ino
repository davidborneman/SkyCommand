//#define __DEBUG__
//#define __WARN__

#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Pin Definitions (Verified: White TX = Pin 4, Green RX = Pin 2 [INT1], NTC150K = Pin A0)
const int txPin = 4;
const int rxPin = 2;
const int thermistorPin = A0;

// TC4 Output Channel Assignment (Valid values: 1 to 4; must be distinct and match Artisan Config -> Device -> ET/BT)
const uint8_t ROASTER_TEMP_CHANNEL = 2; // Channel for the stock roaster probe (BT)
const uint8_t SECOND_PROBE_CHANNEL = 1; // Channel for the secondary NTC150K probe (A0 / ET)

static_assert(ROASTER_TEMP_CHANNEL >= 1 && ROASTER_TEMP_CHANNEL <= 4, "ROASTER_TEMP_CHANNEL must be 1..4");
static_assert(SECOND_PROBE_CHANNEL >= 1 && SECOND_PROBE_CHANNEL <= 4, "SECOND_PROBE_CHANNEL must be 1..4");
static_assert(ROASTER_TEMP_CHANNEL != SECOND_PROBE_CHANNEL, "ROASTER_TEMP_CHANNEL and SECOND_PROBE_CHANNEL must be different");

// NTC150K Thermistor Voltage Divider Constants (5V -> 110k Resistor -> A0 -> Probe -> GND)
const double seriesResistor          = 110000.0; // 110 kOhm fixed resistor (two 220 kOhm in parallel)
const double nominalResistance       = 150000.0; // 150 kOhm at 25 C
const double nominalTemp             = 25.0;     // Reference temp in Celsius
const double bCoefficient            = 3950.0;   // Beta coefficient for OEM NTC150K
const unsigned long PROBE_INTERVAL_MS = 250;     // Non-blocking 250 ms sample interval
const uint8_t ADC_OVERSAMPLE_COUNT   = 8;        // 8-sample average (~0.8 ms) for clean RoR

// Timing Constants (Verified Skywalker V1 Protocol)
const unsigned long one_length   = 1100;  // Threshold for interpreting a bit as '1' (us)
const unsigned long FRAME_GAP_US = 10000; // 10 ms clean HIGH idle gap between TX frames

// Default Startup / Safe-Idle Actuator Values (0 - 100%)
const uint8_t DEFAULT_VENT   = 0;   // OT2: Exhaust Fan off
const uint8_t DEFAULT_FILTER = 0;   // Filter Fan off
const uint8_t DEFAULT_COOL   = 0;   // Cooling Tray Fan off
const uint8_t DEFAULT_DRUM   = 0;   // Drum Motor off
const uint8_t DEFAULT_HEAT   = 0;   // OT1: FIR Heater off
const char    DEFAULT_UNITS  = 'F'; // Default temperature unit ('F' or 'C')

// Buffer Sizes
const int roasterLength    = 7; // Bytes
const int controllerLength = 6; // Bytes

// Buffer Definitions
uint8_t receiveBuffer[roasterLength] = {0};
uint8_t sendBuffer[controllerLength] = {0};

// Interrupt RX Capture Buffers (Pin 2 / INT1)
volatile uint8_t isrBuffer[roasterLength] = {0};
volatile uint8_t isrBitIndex = 0;
volatile bool isrSync = false;
volatile bool rxFrameReady = false;
volatile bool rxPinWasLow = false;
volatile unsigned long rxFallMicros = 0;

// Non-blocking Serial Line Buffer
const size_t SERIAL_BUF_SIZE = 64;
char serialLineBuf[SERIAL_BUF_SIZE];
size_t serialLineLen = 0;
bool serialLineOverflow = false;

// Byte Indices
const int ventByte   = 0;
const int filterByte = 1;
const int coolByte   = 2;
const int drumByte   = 3;
const int heatByte   = 4;
const int checkByte  = 5;

// Temperature State (Stored in Celsius; converted to CorF dynamically in handleREAD)
double stockTempC = 0.0;          // Stock roaster probe in Celsius
double beanTempC  = 0.0;          // Secondary NTC150K probe in Celsius
bool stockTelemetryValid = false; // True once at least one valid checksum frame is decoded
bool beanProbeValid      = false; // True when A0 voltage divider is within valid bounds
unsigned long lastProbeReadMs = 0;
char CorF = DEFAULT_UNITS;

// Link & Failsafe State
const double maxTempC = 300.0; // Over-temperature threshold in Celsius
unsigned long lastEventTime = 0;
const unsigned long lastEventTimeout = 10000000UL; // 10 seconds in microseconds
unsigned long lastFrameEndMicros = 0;
unsigned long offLockoutUntil = 0;
bool roasterLinkActive = false;
bool eStopLatched = false;

// Non-blocking Hardware Interrupt (INT1 on Digital Pin 2)
void rxPinISR() {
  unsigned long now = micros();
  bool isLow = (digitalRead(rxPin) == LOW);

  if (isLow) {
    rxFallMicros = now;
    rxPinWasLow = true;
    return;
  }

  // Ignore spurious rising edges without a tracked falling edge
  if (!rxPinWasLow) return;
  rxPinWasLow = false;

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
  else if (isrSync) {
    if (isrBitIndex < (roasterLength * 8) && lowWidth >= 300 && lowWidth <= 2500) {
      if (lowWidth > one_length) {
        isrBuffer[isrBitIndex / 8] |= (1 << (isrBitIndex % 8));
      }
      isrBitIndex++;

      if (isrBitIndex == (roasterLength * 8)) {
        isrSync = false;
        rxFrameReady = true;
      }
    } else {
      // Abort frame immediately on out-of-spec pulse width to prevent bit-slip corruption
      isrSync = false;
      isrBitIndex = 0;
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

void pulsePinPreamble(int pin, unsigned int durationUs) {
  // Keep global interrupts enabled during the 7.5 ms preamble so INT1 never drops RX edges
  digitalWrite(pin, LOW);
  delayMicroseconds(durationUs);
  digitalWrite(pin, HIGH);
}

void pulsePinDataBit(int pin, unsigned int durationUs) {
  // Keep global interrupts enabled during TX data bits so INT1 captures RX edges without jitter
  digitalWrite(pin, LOW);
  delayMicroseconds(durationUs);
  digitalWrite(pin, HIGH);
}

void sendRoasterMessage() {
  // Send Preamble (~7.5 ms LOW)
  pulsePinPreamble(txPin, 7500);
  delayMicroseconds(3800);

  // Send 6-byte Message (LSB-first: 1500 us = 1, 600 us = 0, 750 us HIGH gap)
  for (int i = 0; i < controllerLength; i++) {
    for (int j = 0; j < 8; j++) {
      if (bitRead(sendBuffer[i], j) == 1) {
        pulsePinDataBit(txPin, 1500); // logic 1
      } else {
        pulsePinDataBit(txPin, 600);  // logic 0
      }
      delayMicroseconds(750);         // inter-bit gap
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
    rxPinWasLow = false;
    interrupts();

    pinMode(txPin, OUTPUT);
    digitalWrite(txPin, HIGH);
    delay(25); // Clean 25 ms HIGH idle level before first preamble

    pinMode(rxPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(rxPin), rxPinISR, CHANGE);

    unsigned long now = micros();
    lastEventTime = now;
    lastFrameEndMicros = now - FRAME_GAP_US; // Unsigned wrap-safe immediate send on loop()
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
    stockTelemetryValid = false;
    stockTempC = 0.0;
    eStopLatched = false;
  }
}

void shutdown() {
  applyDefaults();
  eStopLatched = false;
}

void eStop() {
  eStopLatched = true;
  setValue(&sendBuffer[heatByte], 0);
  setValue(&sendBuffer[ventByte], 100);
}

double formatOutputTemp(double tempCelsius) {
  return (CorF == 'F') ? (tempCelsius * 1.8 + 32.0) : tempCelsius;
}

void sampleNTCProbe() {
  long adcSum = 0;
  for (uint8_t i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
    adcSum += analogRead(thermistorPin);
  }
  double adc = (double)adcSum / (double)ADC_OVERSAMPLE_COUNT;

  if (adc <= 2.0 || adc >= 1021.0) {
    beanProbeValid = false;
    beanTempC = 0.0;
    return;
  }

  double resistance = seriesResistor / ((1023.0 / adc) - 1.0);
  double steinhart = resistance / nominalResistance; // (R / Ro)
  steinhart = log(steinhart);                        // ln(R / Ro)
  steinhart /= bCoefficient;                         // 1/B * ln(R / Ro)
  steinhart += 1.0 / (nominalTemp + 273.15);         // + (1 / To)
  steinhart = 1.0 / steinhart;                       // Invert to Kelvin
  double c = steinhart - 273.15;                     // Convert to Celsius

  if (isnan(c) || c < -20.0 || c > 400.0) {
    beanProbeValid = false;
    beanTempC = 0.0;
    return;
  }

  beanTempC = c;
  beanProbeValid = true;
}

bool updateStockTempFromBuffer() {
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
    return false; // Reject out-of-bounds polynomial artifact
  }

  stockTempC = c;
  stockTelemetryValid = true;
  return true;
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
    updateStockTempFromBuffer();
  }
}

uint8_t clampPercent(uint8_t value) {
  return (value > 100) ? 100 : value;
}

void handleHEAT(uint8_t value) {
  if (eStopLatched) {
    setValue(&sendBuffer[heatByte], 0);
  } else {
    setValue(&sendBuffer[heatByte], clampPercent(value));
  }
  lastEventTime = micros();
}

void handleVENT(uint8_t value) {
  if (eStopLatched) {
    setValue(&sendBuffer[ventByte], 100);
  } else {
    setValue(&sendBuffer[ventByte], clampPercent(value));
  }
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

void handleCHAN(const char* maskParam) {
  char activeMask[5] = {'0', '0', '0', '0', '\0'};
  activeMask[ROASTER_TEMP_CHANNEL - 1] = (char)('0' + ROASTER_TEMP_CHANNEL);
  activeMask[SECOND_PROBE_CHANNEL - 1] = (char)('0' + SECOND_PROBE_CHANNEL);

  if (maskParam != NULL && strlen(maskParam) == 4) {
    Serial.print("# Active channels set to ");
    Serial.println(maskParam);
  } else {
    Serial.print("# Active channels set to ");
    Serial.println(activeMask);
  }
  lastEventTime = micros();
}

void handleREAD() {
  double roasterOut = stockTelemetryValid ? formatOutputTemp(stockTempC) : 0.0;
  double probeOut   = beanProbeValid      ? formatOutputTemp(beanTempC)  : 0.0;

  // Map temperatures onto TC4 Channels 1..4 according to the configured constants
  double ch[4] = {0.0, 0.0, 0.0, 0.0};
  ch[ROASTER_TEMP_CHANNEL - 1] = roasterOut;
  ch[SECOND_PROBE_CHANNEL - 1] = probeOut;

  // Standard TC4 format: Ambient, Ch1, Ch2, [Ch3, Ch4 if used], Heater %, Fan %, 0
  Serial.print(0.0);
  Serial.print(',');
  Serial.print(ch[0]);
  Serial.print(',');
  Serial.print(ch[1]);
  if (ROASTER_TEMP_CHANNEL > 2 || SECOND_PROBE_CHANNEL > 2) {
    Serial.print(',');
    Serial.print(ch[2]);
    Serial.print(',');
    Serial.print(ch[3]);
  }
  Serial.print(',');
  Serial.print(sendBuffer[heatByte]);
  Serial.print(',');
  Serial.print(sendBuffer[ventByte]);
  Serial.println(",0");

  lastEventTime = micros();
}

void dispatchSingleCommand(char* verb, char* paramStr, bool* wakeLinkOut) {
  for (size_t i = 0; verb[i] != '\0'; i++) {
    verb[i] = (char)toupper((unsigned char)verb[i]);
  }

  uint8_t value = 0;
  if (paramStr != NULL) {
    long parsed = strtol(paramStr, NULL, 10);
    if (parsed < 0) parsed = 0;
    if (parsed > 255) parsed = 255;
    value = (uint8_t)parsed;
  }

  if (strcmp(verb, "OFF") == 0) {
    offLockoutUntil = micros() + 1500000UL;
    shutdown();
    disableRoasterLink();
    *wakeLinkOut = false;
  } else if (strcmp(verb, "READ") == 0) {
    handleREAD();
    if ((long)(micros() - offLockoutUntil) < 0) {
      *wakeLinkOut = false;
    }
  } else if (strcmp(verb, "OT1") == 0) {
    handleHEAT(value);
  } else if (strcmp(verb, "OT2") == 0) {
    handleVENT(value);
  } else if (strcmp(verb, "ESTOP") == 0) {
    eStop();
    *wakeLinkOut = false;
  } else if (strcmp(verb, "DRUM") == 0) {
    handleDRUM(value);
  } else if (strcmp(verb, "FILTER") == 0) {
    handleFILTER(value);
  } else if (strcmp(verb, "COOL") == 0) {
    handleCOOL(value);
  } else if (strcmp(verb, "CHAN") == 0) {
    handleCHAN(paramStr);
  } else if (strcmp(verb, "UNITS") == 0) {
    if (paramStr != NULL) {
      char u = (char)toupper((unsigned char)paramStr[0]);
      if (u == 'C' || u == 'F') {
        CorF = u;
      }
    }
    lastEventTime = micros();
  } else {
    *wakeLinkOut = false;
  }
}

void processSerialLine(char* line) {
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\t')) {
    line[--len] = '\0';
  }
  if (len == 0) return;

  bool wakeLink = true;
  char* token = strtok(line, ";");

  while (token != NULL) {
    char* verb = token;
    char* param = NULL;

    char upperCheck[12];
    size_t k = 0;
    while (verb[k] != '\0' && k < sizeof(upperCheck) - 1) {
      upperCheck[k] = (char)toupper((unsigned char)verb[k]);
      k++;
    }
    upperCheck[k] = '\0';

    if (strcmp(upperCheck, "OT1") == 0 || strcmp(upperCheck, "OT2") == 0 ||
        strcmp(upperCheck, "DRUM") == 0 || strcmp(upperCheck, "FILTER") == 0 ||
        strcmp(upperCheck, "COOL") == 0 || strcmp(upperCheck, "CHAN") == 0 ||
        strcmp(upperCheck, "UNITS") == 0) {
      param = strtok(NULL, ";");
    }

    bool stepWake = true;
    dispatchSingleCommand(verb, param, &stepWake);
    if (!stepWake) {
      wakeLink = false;
    }

    token = strtok(NULL, ";");
  }

  if (wakeLink) {
    enableRoasterLink();
  }
}

void pollArtisanSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (!serialLineOverflow) {
        serialLineBuf[serialLineLen] = '\0';
        processSerialLine(serialLineBuf);
      }
      serialLineLen = 0;
      serialLineOverflow = false;
    } else if (!serialLineOverflow) {
      if (serialLineLen < (SERIAL_BUF_SIZE - 1)) {
        serialLineBuf[serialLineLen++] = c;
      } else {
        serialLineOverflow = true;
      }
    }
  }
}

bool itsbeentoolong() {
  return ((micros() - lastEventTime) > lastEventTimeout);
}

bool isTemperatureOverLimit() {
  return ((stockTelemetryValid && !isnan(stockTempC) && stockTempC > maxTempC) ||
          (beanProbeValid && !isnan(beanTempC) && beanTempC > maxTempC));
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

  applyDefaults();

  // Park TX actively HIGH (idle mark state) at boot so roaster stays cleanly asleep until Artisan ON
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);
  pinMode(rxPin, INPUT);

  // Take an initial reading of the NTC150K probe at startup
  sampleNTCProbe();
  lastProbeReadMs = millis();

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

  // Sample secondary NTC150K probe every 250 ms without blocking the 10 ms roaster loop
  if (millis() - lastProbeReadMs >= PROBE_INTERVAL_MS) {
    lastProbeReadMs = millis();
    sampleNTCProbe();
  }

  // Non-blocking Artisan serial command parser
  pollArtisanSerial();
}