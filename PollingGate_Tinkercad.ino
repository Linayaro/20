#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define DECODE_NEC
#define IR_RECEIVE_PIN 8
#define EXCLUDE_UNIVERSAL_PROTOCOLS
#define EXCLUDE_EXOTIC_PROTOCOLS
#include <IRremote.hpp>

// Existing Tinkercad connections (unchanged)
const byte TRIG_PIN      = 10;
const byte ECHO_PIN      = 11;
const byte LED_PIN       = 12;
const byte BUTTON_PIN    = 7;  // physical cancel button
const byte SERVO_PIN     = 9;
const byte PASSAGE_PIN   = 2;  // pushbutton instead of the laser

// Thresholds and timing, as in the latest physical project
const float ENTRY_MIN_CM = 2.0f;
const float ENTRY_MAX_CM = 30.0f;
const unsigned long APPROVAL_TIMEOUT_MS = 10000UL;
const unsigned long PASSAGE_TIMEOUT_MS  = 20000UL;
const unsigned long LASER_CLEAR_MS      = 500UL;
const unsigned long PRESENCE_GRACE_MS   = 400UL;
const unsigned long SENSOR_INTERVAL_MS  = 140UL;
const unsigned long SERVO_MOVE_MS       = 600UL;

// Tinkercad LCD address, NOT the physical LCD's 0x27
LiquidCrystal_I2C lcd(0x20, 16, 2);
Servo gateServo;

// Positional servo in Tinkercad, unlike the physical continuous servo
const byte GATE_CLOSED_ANGLE = 0;
const byte GATE_OPEN_ANGLE   = 90;

enum State { WAITING, APPROVAL, OPENING, GATE_OPEN, CLOSING, WAIT_CLEAR };
int state = WAITING;

unsigned long stateStarted = 0;
unsigned long lastSensorRead = 0;
unsigned long lastPersonSeen = 0;
unsigned long laserClearStarted = 0;
unsigned long lastButtonEvent = 0;
unsigned long entryCount = 0;

bool everSawPerson = false;
bool personPresent = false;
bool previousButtonDown = false;
bool laserInterrupted = false;
bool gateMayBeOpen = false;
bool forcedRestartPending = false;
bool waitUntilEntranceClears = false;

// The real code learns three remote keys instead of guessing their codes.
// Key index: 0 = OK/PLAY-PAUSE, 1 = number 1, 2 = number 9.
// Store the protocol, address and command in one built-in integer.
// Avoid custom parameter types in Tinkercad's generated prototypes.
uint64_t learnedKeys[3];
uint64_t candidateKey = 0;
byte learnStage = 0;
byte matchingPresses = 0;

// Explicit declarations for Arduino/Tinkercad preprocessing.
void queueLine(byte row, const char* text);
void showLCD(const char* line1, const char* line2);
void updateLCD();
void showWaiting();
void showLearning();
void enterState(int next);
void updateUltrasonic();
void finishForcedRestart();
void startClosing(bool forceReset);
void forceCloseAndRestart();
void cancelAction();
void learnRemoteKey(uint64_t key);
void handleRemote();
void handleButton();

// Queue both display lines and write one character per update.
char lcdText[2][17];
byte lcdPosition = 32;
unsigned long lastLCDUpdate = 0;

void queueLine(byte row, const char* text) {
  byte i = 0;
  while (i < 16 && text[i] != '\0') {
    lcdText[row][i] = text[i];
    i++;
  }
  while (i < 16) lcdText[row][i++] = ' ';
  lcdText[row][16] = '\0';
}

void showLCD(const char* line1, const char* line2) {
  queueLine(0, line1);
  queueLine(1, line2);
  lcdPosition = 0;
}

void updateLCD() {
  if (lcdPosition >= 32 || millis() - lastLCDUpdate < 20UL) return;
  byte row = lcdPosition / 16;
  byte col = lcdPosition % 16;
  if (col == 0) lcd.setCursor(0, row);
  lcd.print(lcdText[row][col]);
  lcdPosition++;
  lastLCDUpdate = millis();
}

void showWaiting() {
  char line[20];
  snprintf(line, sizeof(line), "Entries: %lu", entryCount);
  showLCD("Waiting entry", line);
}

void showLearning() {
  switch (learnStage) {
    case 0:
      showLCD("IR setup: PLAY",
              matchingPresses ? "Press PLAY again" : "Press PLAY once");
      break;
    case 1:
      showLCD("IR setup: 1",
              matchingPresses ? "Press 1 again" : "Press 1 once");
      break;
    case 2:
      showLCD("IR setup: 9",
              matchingPresses ? "Press 9 again" : "Press 9 once");
      break;
  }
}

void enterState(int next) {
  state = next;
  stateStarted = millis();

  switch (state) {
    case WAITING:
      gateServo.write(GATE_CLOSED_ANGLE);
      if (waitUntilEntranceClears)
        showLCD("Reset complete", "Clear entrance");
      else
        showWaiting();
      Serial.println(F("State: WAITING"));
      break;

    case APPROVAL:
      showLCD("Person detected", "Press PLAY/PAUSE");
      Serial.println(F("State: APPROVAL - press OK"));
      break;

    case OPENING:
      gateMayBeOpen = true;
      gateServo.write(GATE_OPEN_ANGLE);
      showLCD("Opening gate", "Please wait");
      Serial.println(F("State: OPENING"));
      break;

    case GATE_OPEN:
      laserInterrupted = false;
      laserClearStarted = 0;
      showLCD("Gate open", "D2 press+release");
      Serial.println(F("State: GATE_OPEN"));
      break;

    case CLOSING:
      gateServo.write(GATE_CLOSED_ANGLE);
      showLCD("Closing gate", "Please wait");
      Serial.println(F("State: CLOSING"));
      break;

    case WAIT_CLEAR:
      gateMayBeOpen = false;
      showLCD("Cycle finished", "Clear entrance");
      Serial.println(F("State: WAIT_CLEAR"));
      break;
  }
}

void updateUltrasonic() {
  if (millis() - lastSensorRead < SENSOR_INTERVAL_MS) return;
  lastSensorRead = millis();

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

  bool presentNow = everSawPerson &&
                    (millis() - lastPersonSeen <= PRESENCE_GRACE_MS);
  if (presentNow != personPresent) {
    personPresent = presentNow;
    digitalWrite(LED_PIN, personPresent ? HIGH : LOW);
    Serial.println(personPresent ? F("Person detected")
                                 : F("Entrance clear"));
  }
}

void finishForcedRestart() {
  forcedRestartPending = false;
  gateMayBeOpen = false;
  laserInterrupted = false;
  laserClearStarted = 0;
  waitUntilEntranceClears = personPresent;
  enterState(WAITING);
  Serial.println(F("Button 9: reset complete; counter kept"));
}

void startClosing(bool forceReset) {
  forcedRestartPending = forceReset;
  enterState(CLOSING);
}

// Key 9: CLOSE AND RESET from any active state, even if D2 is pressed.
// It does NOT erase the accumulated entry count.
void forceCloseAndRestart() {
  Serial.println(F("Button 9: FORCE CLOSE / RESTART"));
  gateServo.write(GATE_CLOSED_ANGLE);

  if (state == OPENING || state == GATE_OPEN || state == CLOSING) {
    startClosing(true);
  } else {
    finishForcedRestart();
  }
}

// Both remote 1 and physical D7 call exactly this function.
void cancelAction() {
  if (state == APPROVAL) {
    Serial.println(F("Action cancelled"));
    enterState(WAIT_CLEAR);
  } else if (state == GATE_OPEN) {
    // Button D2 held down = person still inside passage: no manual close.
    if (digitalRead(PASSAGE_PIN) == HIGH) {
      Serial.println(F("Manual close; no entry counted"));
      startClosing(false);
    } else {
      Serial.println(F("Cancel blocked: passage occupied"));
    }
  } else {
    Serial.println(F("Cancel ignored in current state"));
  }
}

void learnRemoteKey(uint64_t key) {
  for (byte i = 0; i < learnStage; i++) {
    if (key == learnedKeys[i]) {
      Serial.println(F("Use a different remote button"));
      return;
    }
  }

  if (matchingPresses == 0 || key != candidateKey) {
    candidateKey = key;
    matchingPresses = 1;
    Serial.println(F("First press saved; press same button again"));
    showLearning();
    return;
  }

  learnedKeys[learnStage] = key;
  matchingPresses = 0;
  Serial.print(F("Learned key index: "));
  Serial.println(learnStage);
  learnStage++;

  if (learnStage == 3) {
    Serial.println(F("IR ready: OK=approve, 1=cancel, 9=reset"));
    enterState(WAITING);
  } else {
    showLearning();
  }
}

void handleRemote() {
  if (!IrReceiver.decode()) return;

  byte protocol = (byte)IrReceiver.decodedIRData.protocol;
  uint16_t address = IrReceiver.decodedIRData.address;
  uint16_t command = IrReceiver.decodedIRData.command;
  uint64_t key = ((uint64_t)protocol << 32) |
                 ((uint64_t)address << 16) | command;
  byte flags = IrReceiver.decodedIRData.flags;
  bool unknown = IrReceiver.decodedIRData.protocol == UNKNOWN;
  IrReceiver.resume();

  if (unknown || command == 0 ||
      (flags & IRDATA_FLAGS_IS_REPEAT)) return;

  Serial.print(F("IR command: 0x"));
  Serial.println(command, HEX);

  if (learnStage < 3) {
    learnRemoteKey(key);
  } else if (key == learnedKeys[2]) {
    forceCloseAndRestart();
  } else if (key == learnedKeys[1]) {
    Serial.println(F("Remote 1: cancel"));
    cancelAction();
  } else if (key == learnedKeys[0]) {
    if (state == APPROVAL) {
      Serial.println(F("Remote OK: approved"));
      enterState(OPENING);
    } else {
      Serial.println(F("OK ignored: not in APPROVAL"));
    }
  } else {
    Serial.println(F("Unassigned remote key"));
  }
}

void handleButton() {
  bool down = digitalRead(BUTTON_PIN) == LOW;
  if (down && !previousButtonDown &&
      millis() - lastButtonEvent >= 150UL) {
    lastButtonEvent = millis();
    Serial.println(F("Physical D7: cancel"));
    cancelAction();
  }
  previousButtonDown = down;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PASSAGE_PIN, INPUT_PULLUP);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  gateServo.attach(SERVO_PIN);
  gateServo.write(GATE_CLOSED_ANGLE);

  lcd.init();
  lcd.backlight();
  showLCD("IR setup", "Please wait");

  IrReceiver.begin(IR_RECEIVE_PIN);
  Serial.println(F("Learn PLAY, 1, 9: two short presses each"));
  showLearning();
}

void loop() {
  handleRemote();
  handleButton();
  updateUltrasonic();
  handleRemote();

  // No entry sequence before all three buttons have been learned.
  if (learnStage < 3) {
    updateLCD();
    return;
  }

  unsigned long now = millis();
  bool passageButtonDown = digitalRead(PASSAGE_PIN) == LOW;

  switch (state) {
    case WAITING:
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
      // Simulation of the real laser: press = blocked; release = clear.
      if (passageButtonDown) {
        if (!laserInterrupted)
          Serial.println(F("Sim laser: passage detected (D2 down)"));
        laserInterrupted = true;
        laserClearStarted = 0;
      }

      if (laserInterrupted) {
        if (!passageButtonDown) {
          if (laserClearStarted == 0) laserClearStarted = now;
          if (now - laserClearStarted >= LASER_CLEAR_MS) {
            entryCount++;
            Serial.print(F("Entry counted. Total: "));
            Serial.println(entryCount);
            startClosing(false);
          }
        } else {
          laserClearStarted = 0;
        }
      } else if (now - stateStarted >= PASSAGE_TIMEOUT_MS &&
                 !passageButtonDown) {
        Serial.println(F("No passage in 20s: closing without count"));
        startClosing(false);
      }
      break;

    case CLOSING:
      if (now - stateStarted >= SERVO_MOVE_MS) {
        if (forcedRestartPending) finishForcedRestart();
        else enterState(WAIT_CLEAR);
      }
      break;

    case WAIT_CLEAR:
      if (!personPresent) enterState(WAITING);
      break;
  }
  updateLCD();
  delay(2);
}