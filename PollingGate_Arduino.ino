#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VL53L1X.h>
#include <Servo.h>

#define IR_RECEIVE_PIN 8
#include <IRremote.hpp>

// Arduino Uno - physical wiring
const byte TRIG_PIN   = 10;
const byte ECHO_PIN   = 11;
const byte LED_PIN    = 12;
const byte BUTTON_PIN = 7;
const byte SERVO_PIN  = 9;

// Distance thresholds (cm): adjust after installation
const float ENTRY_MIN_CM      = 2.0f;
const float ENTRY_MAX_CM      = 30.0f;
const float LASER_DETECT_CM   = 30.0f;
const float LASER_CLEAR_CM    = 40.0f;

// Continuous-rotation servo: direction / stop / approximate travel time
const byte SERVO_OPEN  = 110;
const byte SERVO_CLOSE = 70;
const byte SERVO_STOP  = 90;
const unsigned long SERVO_MOVE_MS       = 600UL;
const unsigned long APPROVAL_TIMEOUT_MS = 10000UL;
const unsigned long PASSAGE_TIMEOUT_MS  = 20000UL;
const unsigned long LASER_CLEAR_MS      = 500UL;
const unsigned long PRESENCE_GRACE_MS   = 400UL;
const unsigned long SENSOR_INTERVAL_MS  = 140UL;

LiquidCrystal_I2C lcd(0x27, 16, 2);
VL53L1X laser;
Servo gateServo;

enum State { WAITING, APPROVAL, OPENING, GATE_OPEN, CLOSING, WAIT_CLEAR };
State state = WAITING;

unsigned long stateStarted = 0;
unsigned long lastUltrasonicRead = 0;
unsigned long lastLaserRead = 0;
unsigned long lastPersonSeen = 0;
unsigned long laserClearStarted = 0;
unsigned long lastButtonEvent = 0;
unsigned long entries = 0;
unsigned long closeDurationMs = SERVO_MOVE_MS;

bool everSawPerson = false;
bool personPresent = false;
bool previousButtonDown = false;
bool laserValid = false;
bool laserInterrupted = false;
bool gateMayBeOpen = false;
bool forcedRestartPending = false;
bool waitUntilEntranceClears = false;
float laserDistanceCM = 0.0f;

// Learn the remote buttons, rather than guessing their IR codes.
// 0 = OK (approve); 1 = number 1 (cancel); 2 = number 9 (force reset).
struct RemoteKey {
  uint8_t protocol;
  uint16_t address;
  uint16_t command;
};
RemoteKey learnedKeys[3];
RemoteKey candidateKey;
byte learnStage = 0;
byte matchingPresses = 0;

bool sameKey(const RemoteKey &a, const RemoteKey &b) {
  return a.protocol == b.protocol &&
         a.address == b.address &&
         a.command == b.command;
}

void showLCD(const __FlashStringHelper *line1,
             const __FlashStringHelper *line2) {
  lcd.setCursor(0, 0);
  lcd.print(F("                "));
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void showWaiting() {
  showLCD(F("Waiting entry"), F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Entries: "));
  lcd.print(entries);
}

void showLearning() {
  switch (learnStage) {
    case 0:
      showLCD(F("IR setup: OK"),
              matchingPresses ? F("Press OK again") : F("Press OK once"));
      break;
    case 1:
      showLCD(F("IR setup: 1"),
              matchingPresses ? F("Press 1 again") : F("Press 1 once"));
      break;
    case 2:
      showLCD(F("IR setup: 9"),
              matchingPresses ? F("Press 9 again") : F("Press 9 once"));
      break;
  }
}

void enterState(State nextState) {
  state = nextState;
  stateStarted = millis();

  switch (state) {
    case WAITING:
      gateServo.write(SERVO_STOP);
      if (waitUntilEntranceClears)
        showLCD(F("Reset complete"), F("Clear entrance"));
      else
        showWaiting();
      Serial.println(F("State: WAITING"));
      break;

    case APPROVAL:
      showLCD(F("Person detected"), F("Press OK on IR"));
      Serial.println(F("State: APPROVAL - press OK"));
      break;

    case OPENING:
      gateMayBeOpen = true;
      gateServo.write(SERVO_OPEN);
      showLCD(F("Opening gate"), F("Please wait"));
      Serial.println(F("State: OPENING"));
      break;

    case GATE_OPEN:
      gateServo.write(SERVO_STOP);
      laserInterrupted = false;
      laserClearStarted = 0;
      showLCD(F("Gate open"), F("Pass laser area"));
      Serial.println(F("State: GATE_OPEN"));
      break;

    case CLOSING:
      gateServo.write(SERVO_CLOSE);
      showLCD(F("Closing gate"), F("Please wait"));
      Serial.println(F("State: CLOSING"));
      break;

    case WAIT_CLEAR:
      gateServo.write(SERVO_STOP);
      gateMayBeOpen = false;
      showLCD(F("Cycle finished"), F("Clear entrance"));
      Serial.println(F("State: WAIT_CLEAR"));
      break;
  }
}

void updateUltrasonic() {
  if (millis() - lastUltrasonicRead < SENSOR_INTERVAL_MS) return;
  lastUltrasonicRead = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echo = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (echo != 0) {
    float distanceCM = echo * 0.0343f / 2.0f;
    if (distanceCM >= ENTRY_MIN_CM && distanceCM <= ENTRY_MAX_CM) {
      lastPersonSeen = millis();
      everSawPerson = true;
    }
  }

  bool newPresence = everSawPerson &&
                     millis() - lastPersonSeen <= PRESENCE_GRACE_MS;
  if (newPresence != personPresent) {
    personPresent = newPresence;
    digitalWrite(LED_PIN, personPresent ? HIGH : LOW);
    Serial.println(personPresent ? F("Person detected") : F("Entrance clear"));
  }
}

void updateLaser() {
  if (millis() - lastLaserRead < SENSOR_INTERVAL_MS) return;
  lastLaserRead = millis();

  uint16_t distanceMM = laser.read();
  laserValid = !laser.timeoutOccurred() && distanceMM > 0;
  if (laserValid) laserDistanceCM = distanceMM / 10.0f;
}

void finishForcedRestart() {
  forcedRestartPending = false;
  gateMayBeOpen = false;
  laserInterrupted = false;
  laserClearStarted = 0;
  waitUntilEntranceClears = personPresent;
  enterState(WAITING);
  Serial.println(F("Button 9: reset completed; entry counter kept"));
}

void startClosing(bool forceRestart = false) {
  forcedRestartPending = forceRestart;
  closeDurationMs = SERVO_MOVE_MS;
  enterState(CLOSING);
}

// Button 9 is an unconditional *demo override*:
// close even if the laser sees an obstacle, then restart the entry cycle.
// Test only with the moving arm detached and with no one in its path.
void forceCloseAndRestart() {
  Serial.println(F("Button 9: FORCE CLOSE / RESTART"));

  if (!gateMayBeOpen) {
    gateServo.write(SERVO_STOP); // Already closed in the software model
    finishForcedRestart();
    return;
  }

  unsigned long remaining = SERVO_MOVE_MS;
  unsigned long elapsed = millis() - stateStarted;

  if (state == OPENING) {
    // Reverse only the approximate amount already opened.
    remaining = (elapsed < SERVO_MOVE_MS) ? elapsed : SERVO_MOVE_MS;
  } else if (state == CLOSING) {
    // Finish the ongoing closing motion; do not overdrive the arm.
    remaining = (elapsed < closeDurationMs) ?
                closeDurationMs - elapsed : 0;
  }

  if (remaining == 0) {
    gateServo.write(SERVO_STOP);
    finishForcedRestart();
    return;
  }

  forcedRestartPending = true;
  closeDurationMs = remaining;
  enterState(CLOSING);
}

// Both physical D7 and remote 1 execute exactly this function.
void cancelAction() {
  if (state == APPROVAL) {
    Serial.println(F("Action cancelled"));
    enterState(WAIT_CLEAR);
  } else if (state == GATE_OPEN) {
    if (laserValid && laserDistanceCM >= LASER_CLEAR_CM) {
      Serial.println(F("Manual close, no entry counted"));
      startClosing();
    } else {
      Serial.println(F("Cancel blocked: laser area not clear"));
    }
  } else {
    Serial.println(F("Cancel ignored in current state"));
  }
}

void learnRemoteKey(const RemoteKey &key) {
  for (byte i = 0; i < learnStage; i++) {
    if (sameKey(key, learnedKeys[i])) {
      Serial.println(F("Use a different button for this action"));
      return;
    }
  }

  if (matchingPresses == 0 || !sameKey(key, candidateKey)) {
    candidateKey = key;
    matchingPresses = 1;
    Serial.println(F("First valid press saved; press same button again"));
    showLearning();
    return;
  }

  learnedKeys[learnStage] = key;
  matchingPresses = 0;
  Serial.print(F("Learned key number: "));
  Serial.println(learnStage);
  learnStage++;

  if (learnStage == 3) {
    Serial.println(F("IR ready: OK=approve, 1=cancel, 9=force close/reset"));
    enterState(WAITING);
  } else {
    showLearning();
  }
}

void handleRemote() {
  if (!IrReceiver.decode()) return;

  RemoteKey key;
  key.protocol = (uint8_t)IrReceiver.decodedIRData.protocol;
  key.address = IrReceiver.decodedIRData.address;
  key.command = IrReceiver.decodedIRData.command;
  uint8_t flags = IrReceiver.decodedIRData.flags;
  bool unknown = IrReceiver.decodedIRData.protocol == UNKNOWN;
  IrReceiver.resume();

  // Reject noise, failed decoding, zero commands and held-key repeats.
  if (unknown || key.command == 0 ||
      (flags & IRDATA_FLAGS_IS_REPEAT) ||
      (flags & IRDATA_FLAGS_WAS_OVERFLOW)) {
    return;
  }

  Serial.print(F("IR command: 0x"));
  Serial.println(key.command, HEX);

  if (learnStage < 3) {
    learnRemoteKey(key);
    return;
  }

  if (sameKey(key, learnedKeys[2])) {
    forceCloseAndRestart();
  } else if (sameKey(key, learnedKeys[1])) {
    Serial.println(F("Remote 1: cancel"));
    cancelAction();
  } else if (sameKey(key, learnedKeys[0])) {
    if (state == APPROVAL) {
      Serial.println(F("Remote OK: approved"));
      enterState(OPENING);
    } else {
      Serial.println(F("OK ignored: system not in APPROVAL"));
    }
  } else {
    Serial.println(F("IR key not assigned"));
  }
}

void handleButton() {
  bool buttonDown = digitalRead(BUTTON_PIN) == LOW;
  if (buttonDown && !previousButtonDown &&
      millis() - lastButtonEvent >= 150UL) {
    lastButtonEvent = millis();
    Serial.println(F("Physical D7: cancel"));
    cancelAction();
  }
  previousButtonDown = buttonDown;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  gateServo.attach(SERVO_PIN);
  gateServo.write(SERVO_STOP);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  showLCD(F("Initializing"), F("Laser and IR"));

  laser.setTimeout(500);
  if (!laser.init()) {
    showLCD(F("Laser error"), F("Check 0x29"));
    Serial.println(F("Laser initialization FAILED"));
    while (true) delay(1000);
  }
  laser.setDistanceMode(VL53L1X::Long);
  laser.setMeasurementTimingBudget(50000);
  laser.startContinuous(100);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println(F("Ready to learn OK, 1, 9 (two short presses each)"));
  showLearning();
}

void loop() {
  // Handle remote early so normal measurements do not delay reactions.
  handleRemote();
  handleButton();
  updateUltrasonic();
  updateLaser();
  handleRemote();

  if (learnStage < 3) return;

  unsigned long now = millis();
  switch (state) {
    case WAITING:
      // After button 9, require the previous person to leave first.
      if (waitUntilEntranceClears) {
        if (!personPresent) {
          waitUntilEntranceClears = false;
          showWaiting();
          Serial.println(F("Ready for next person"));
        }
      } else if (personPresent) {
        enterState(APPROVAL);
      }
      break;

    case APPROVAL:
      if (now - stateStarted >= APPROVAL_TIMEOUT_MS) {
        Serial.println(F("Approval timed out"));
        enterState(WAIT_CLEAR);
      }
      break;

    case OPENING:
      if (now - stateStarted >= SERVO_MOVE_MS) {
        enterState(GATE_OPEN);
      }
      break;

    case GATE_OPEN:
      if (laserValid && laserDistanceCM <= LASER_DETECT_CM) {
        if (!laserInterrupted) Serial.println(F("Laser: passage detected"));
        laserInterrupted = true;
        laserClearStarted = 0;
      }

      if (laserInterrupted) {
        if (laserValid && laserDistanceCM >= LASER_CLEAR_CM) {
          if (laserClearStarted == 0) laserClearStarted = now;
          if (now - laserClearStarted >= LASER_CLEAR_MS) {
            entries++;
            Serial.print(F("Entry counted. Total: "));
            Serial.println(entries);
            startClosing();
          }
        } else {
          laserClearStarted = 0;
        }
      } else if (now - stateStarted >= PASSAGE_TIMEOUT_MS &&
                 laserValid && laserDistanceCM >= LASER_CLEAR_CM) {
        Serial.println(F("No passage: closing without counting"));
        startClosing();
      }
      break;

    case CLOSING:
      if (now - stateStarted >= closeDurationMs) {
        gateServo.write(SERVO_STOP);
        gateMayBeOpen = false;
        if (forcedRestartPending) finishForcedRestart();
        else enterState(WAIT_CLEAR);
      }
      break;

    case WAIT_CLEAR:
      if (!personPresent) enterState(WAITING);
      break;
  }
}
