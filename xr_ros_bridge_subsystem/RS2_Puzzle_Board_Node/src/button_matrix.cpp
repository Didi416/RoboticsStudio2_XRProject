#include <button_matrix.h>
#include <game_state.h>

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {3, 8, 7, 5};
byte colPins[COLS] = {4, 2, 6};
Keypad kpd = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

String correct_code = "1234";
String entered_code = "";
int num_cursor = 0;

// RGB LED
int redPin = 11;
int greenPin = 10;
int bluePin = 9;

// 14-seg display
Adafruit_AlphaNum4 disp = Adafruit_AlphaNum4();

// ── Display helpers ──────────────────────────────────────────

void dispWrite(char a, char b, char c, char d) {
  disp.writeDigitAscii(0, a);
  disp.writeDigitAscii(1, b);
  disp.writeDigitAscii(2, c);
  disp.writeDigitAscii(3, d);
  disp.writeDisplay();
}

// Default idle state: "----"
void dispDashes() {
  dispWrite('-', '-', '-', '-');
}

// Show digits entered so far, pad remaining with dashes
// e.g. 2 digits entered → "12--"
void dispCode() {
  char slots[4] = {'-', '-', '-', '-'};
  for (int i = 0; i < num_cursor && i < 4; i++) {
    slots[i] = entered_code[i];
  }
  dispWrite(slots[0], slots[1], slots[2], slots[3]);
}

// Show "XXXX" for incorrect or overflow
void dispError() {
  dispWrite('X', 'X', 'X', 'X');
}

// Show "GOOD" for correct
void dispGood() {
  dispWrite('G', 'O', 'O', 'D');
  puzzleCodeSolved = true;
}

// ── Existing helpers ─────────────────────────────────────────

void setColour(bool red, bool green, bool blue) {
  digitalWrite(redPin, red);
  digitalWrite(greenPin, green);
  digitalWrite(bluePin, blue);
}

void resetEntry() {
  entered_code = "";
  num_cursor = 0;
  dispDashes();  // back to "----" on reset
}

// ── Setup / loop ─────────────────────────────────────────────

void button_matrix_setup() {
  kpd.setDebounceTime(50);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  setColour(false, false, true);  // idle: blue

  disp.begin(0x71);
  disp.setBrightness(4);
  dispDashes();  // start with "----"
}

void button_matrix_work() {
  if (kpd.getKeys())
  {
    for (int i = 0; i < LIST_MAX; i++)
    {
      if (kpd.key[i].stateChanged && kpd.key[i].kstate == PRESSED)
      {
        char k = kpd.key[i].kchar;

        if (k == '*') {
          // clear/reset
          Serial.println("Cleared.");
          setColour(false, false, true);
          resetEntry();  // resets display to "----"
        }
        else if (k == '#') {
          // submit
          if (entered_code == String(generatedCode)) {
            Serial.println("Code " + entered_code + " -> Correct! :)");
            setColour(false, true, false);  // green
            dispGood();                     // "GOOD"
            delay(2000);
          } else {
            Serial.println("Code " + entered_code + " -> Incorrect :(");
            setColour(true, false, false);  // red
            dispError();                    // "XXXX"
            delay(1000);
          }
          // leave display showing result until next keypress
          resetEntry();
        }
        else {
          // numeric digit
          if (num_cursor >= 4) {
            Serial.println("Max 4 digits. Press # to submit or * to clear.");
            setColour(true, false, false);
            dispError();  // "XXXX" — too many inputs
            delay(1000);
            resetEntry();
          } else {
            entered_code.concat(k);
            num_cursor++;
            // Serial.println("Entered so far: " + entered_code);
            setColour(false, false, true);  // stay blue while typing
            dispCode();   // show digits entered so far, pad with "--"
          }
        }
      }
    }
  }
}

// ── Helper functions for state serialization ──────────────────
String getButtonStates() {
  // Returns button states as comma-separated values (12 total)
  // Keys are ordered by their position: 1,2,3,4,5,6,7,8,9,*,0,#
  String states = "";
  for (int i = 0; i < LIST_MAX; i++) {
    if (kpd.key[i].kstate == PRESSED) {
      states += "1";
    } else {
      states += "0";
    }
    if (i < LIST_MAX - 1) states += ",";
  }
  return states;
}

String getDisplayText() {
  // Returns current display text (4 characters)
  // This captures the text being entered
  String displayText = entered_code;
  if (displayText.length() == 0) {
    displayText = "----";
  }
  // Pad to 4 characters with dashes
  while (displayText.length() < 4) {
    displayText += "-";
  }
  if (displayText.length() > 4) {
    displayText = displayText.substring(0, 4);
  }
  return displayText;
}