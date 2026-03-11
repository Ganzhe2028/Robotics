#define PIN_BTN_SELECT 0
#define PIN_BTN_START 6

#define BTN_RELEASED HIGH
#define BTN_PRESSED LOW

const unsigned long BUTTON_DEBOUNCE_MS = 30;
const unsigned long STATUS_PRINT_MS = 1000;
const unsigned long STUCK_LOW_WARN_MS = 1500;

enum ButtonId {
  BUTTON_START,
  BUTTON_SELECT
};

struct ButtonState {
  uint8_t pin;
  const char *name;
  bool stableLevel;
  bool lastReading;
  bool pressedArmed;
  bool stuckLowWarned;
  unsigned long stableSinceTime;
  unsigned long lastChangeTime;
};

ButtonState startButton = {
  PIN_BTN_START, "START", BTN_RELEASED, BTN_RELEASED, false, false, 0, 0
};
ButtonState selectButton = {
  PIN_BTN_SELECT, "SELECT", BTN_RELEASED, BTN_RELEASED, false, false, 0, 0
};

const ButtonId EXPECTED_SEQUENCE[] = {
  BUTTON_START,
  BUTTON_SELECT,
  BUTTON_SELECT,
  BUTTON_START
};

const uint8_t EXPECTED_SEQUENCE_LENGTH =
  sizeof(EXPECTED_SEQUENCE) / sizeof(EXPECTED_SEQUENCE[0]);

uint8_t sequenceIndex = 0;
unsigned long lastStatusPrintTime = 0;
unsigned long startPressCount = 0;
unsigned long selectPressCount = 0;

const char *buttonName(ButtonId buttonId) {
  return buttonId == BUTTON_START ? "START" : "SELECT";
}

const char *levelName(bool level) {
  return level == BTN_PRESSED ? "LOW/PRESSED" : "HIGH/RELEASED";
}

void printPrefix() {
  Serial.print('[');
  Serial.print(millis());
  Serial.print(F(" ms] "));
}

void setupButton(ButtonState &button) {
  pinMode(button.pin, INPUT_PULLUP);
  button.stableLevel = digitalRead(button.pin);
  button.lastReading = button.stableLevel;
  button.pressedArmed = false;
  button.stuckLowWarned = false;
  button.stableSinceTime = millis();
  button.lastChangeTime = millis();
}

void printSequenceGuide() {
  Serial.println(F("[GUIDE] expected sequence: START + SELECT + SELECT + START"));
}

void printButtonBootState(const ButtonState &button) {
  Serial.print(F("[BOOT] "));
  Serial.print(button.name);
  Serial.print(F(" initial state = "));
  Serial.println(levelName(button.stableLevel));
}

void handleReleasedButton(ButtonId buttonId) {
  if (buttonId == BUTTON_START) {
    startPressCount++;
  } else {
    selectPressCount++;
  }

  printPrefix();
  Serial.print(F("[SEQ] input="));
  Serial.print(buttonName(buttonId));
  Serial.print(F(" expected="));
  Serial.println(buttonName(EXPECTED_SEQUENCE[sequenceIndex]));

  if (buttonId == EXPECTED_SEQUENCE[sequenceIndex]) {
    sequenceIndex++;

    printPrefix();
    Serial.print(F("[SEQ] progress "));
    Serial.print(sequenceIndex);
    Serial.print('/');
    Serial.println(EXPECTED_SEQUENCE_LENGTH);

    if (sequenceIndex >= EXPECTED_SEQUENCE_LENGTH) {
      printPrefix();
      Serial.println(F("[SEQ] PASS"));
      sequenceIndex = 0;
      printSequenceGuide();
    }
    return;
  }

  if (buttonId == EXPECTED_SEQUENCE[0]) {
    sequenceIndex = 1;
    printPrefix();
    Serial.println(F("[SEQ] restart from step 1"));
    return;
  }

  sequenceIndex = 0;
  printPrefix();
  Serial.println(F("[SEQ] reset"));
  printSequenceGuide();
}

bool updateButton(ButtonState &button, ButtonId buttonId) {
  bool reading = digitalRead(button.pin);

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeTime = millis();

    printPrefix();
    Serial.print(F("[RAW] "));
    Serial.print(button.name);
    Serial.print(F(" -> "));
    Serial.println(levelName(reading));
  }

  if (millis() - button.lastChangeTime < BUTTON_DEBOUNCE_MS) {
    return false;
  }

  if (reading == button.stableLevel) {
    return false;
  }

  button.stableLevel = reading;
  button.stableSinceTime = millis();
  button.stuckLowWarned = false;

  printPrefix();
  Serial.print(F("[BTN] "));
  Serial.print(button.name);
  Serial.print(button.stableLevel == BTN_PRESSED ? F(" PRESSED ") : F(" RELEASED "));
  Serial.print(F("("));
  Serial.print(levelName(button.stableLevel));
  Serial.println(F(")"));

  if (button.stableLevel == BTN_PRESSED) {
    button.pressedArmed = true;
    return false;
  }

  if (!button.pressedArmed) {
    return false;
  }

  button.pressedArmed = false;
  handleReleasedButton(buttonId);
  return true;
}

void warnIfButtonStuck(ButtonState &button) {
  if (button.stableLevel != BTN_PRESSED) {
    return;
  }

  if (button.pressedArmed) {
    return;
  }

  if (button.stuckLowWarned) {
    return;
  }

  if (millis() - button.stableSinceTime < STUCK_LOW_WARN_MS) {
    return;
  }

  button.stuckLowWarned = true;

  printPrefix();
  Serial.print(F("[WARN] "));
  Serial.print(button.name);
  Serial.println(F(" held LOW without an edge; check wiring, switch, or pin choice"));
}

void printStatus() {
  if (millis() - lastStatusPrintTime < STATUS_PRINT_MS) {
    return;
  }

  lastStatusPrintTime = millis();

  printPrefix();
  Serial.print(F("[STATUS] START="));
  Serial.print(levelName(startButton.stableLevel));
  Serial.print(F(" SELECT="));
  Serial.print(levelName(selectButton.stableLevel));
  Serial.print(F(" next="));
  Serial.print(sequenceIndex + 1);
  Serial.print('/');
  Serial.print(EXPECTED_SEQUENCE_LENGTH);
  Serial.print(F(" count(start/select)="));
  Serial.print(startPressCount);
  Serial.print('/');
  Serial.print(selectPressCount);
  Serial.print(F(" armed(start/select)="));
  Serial.print(startButton.pressedArmed ? 1 : 0);
  Serial.print('/');
  Serial.println(selectButton.pressedArmed ? 1 : 0);
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] button diagnostic"));
  Serial.print(F("[BOOT] START pin="));
  Serial.print(PIN_BTN_START);
  Serial.print(F(" SELECT pin="));
  Serial.println(PIN_BTN_SELECT);
  Serial.println(F("[BOOT] wiring: use INPUT_PULLUP, pressed = pin shorted to GND"));
  Serial.println(F("[BOOT] counting happens on stable release"));

  setupButton(startButton);
  setupButton(selectButton);
  printButtonBootState(startButton);
  printButtonBootState(selectButton);
  printSequenceGuide();
}

void loop() {
  updateButton(startButton, BUTTON_START);
  updateButton(selectButton, BUTTON_SELECT);
  warnIfButtonStuck(startButton);
  warnIfButtonStuck(selectButton);
  printStatus();
  delay(1);
}
