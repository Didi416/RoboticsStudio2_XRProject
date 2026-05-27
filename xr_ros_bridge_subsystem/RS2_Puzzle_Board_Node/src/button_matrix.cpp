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
String current_display = "";
int num_cursor = 0;

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
  current_display = entered_code;
  dispWrite(slots[0], slots[1], slots[2], slots[3]);
}

// Show "XXXX" for incorrect or overflow
void dispError() {
  dispWrite('X', 'X', 'X', 'X');
  current_display = "XXXX";
}

// Show "GOOD" for correct
void dispGood() {
  dispWrite('G', 'O', 'O', 'D');
  puzzleCodeSolved = true;
  current_display = "GOOD";
}

void resetEntry() {
  entered_code = "";
  num_cursor = 0;
  current_display = entered_code;
  dispDashes();  // back to "----" on reset
}

// ── Setup / loop ─────────────────────────────────────────────

void button_matrix_setup() {
  kpd.setDebounceTime(50);
  setRGB(false, false, true);  // idle: blue

  disp.begin(0x71);
  disp.setBrightness(4);
  dispDashes();  // start with "----"
}

// void button_matrix_work() {
//   if (kpd.getKeys())
//   {
//     for (int i = 0; i < LIST_MAX; i++)
//     {
//       if (kpd.key[i].stateChanged && kpd.key[i].kstate == PRESSED)
//       {
//         char k = kpd.key[i].kchar;
//         if (k == '*') {
//           // clear/reset
//           Serial.println("Cleared.");
//           setRGB(false, false, true);
//           resetEntry();  // resets display to "----"
//         }
//         else if (k == '#') {
//           // submit
//           if (entered_code == String(generatedCode)) {
//             Serial.println("Code " + entered_code + " -> Correct! :)");
//             setRGB(false, true, false);  // green
//             dispGood();                     // "GOOD"
//             delay(2000);
//           } else {
//             Serial.println("Code " + entered_code + " -> Incorrect :(");
//             setRGB(true, false, false);  // red
//             dispError();                    // "XXXX"
//             delay(1000);
//           }
//           // leave display showing result until next keypress
//           resetEntry();
//         }
//         else {
//           // numeric digit
//           if (num_cursor >= 4) {
//             Serial.println("Max 4 digits. Press # to submit or * to clear.");
//             setRGB(true, false, false);
//             dispError();  // "XXXX" — too many inputs
//             delay(1000);
//             resetEntry();
//           } else {
//             entered_code.concat(k);
//             num_cursor++;
//             // Serial.println("Entered so far: " + entered_code);
//             setRGB(false, false, true);  // stay blue while typing
//             dispCode();   // show digits entered so far, pad with "--"
//           }
//         }
//       }
//     }
//   }
// }

// Global variables to track display timeout state
unsigned long display_timeout = 0;
const unsigned long CORRECT_DISPLAY_TIME = 2000;  // 2000ms for correct code
const unsigned long ERROR_DISPLAY_TIME = 1000;     // 1000ms for errors

void button_matrix_work() {
  // Check if we're in a display timeout and should reset
  if (display_timeout > 0 && millis() >= display_timeout) {
    display_timeout = 0;
    resetEntry();
  }

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
          setRGB(false, false, true);
          resetEntry();  // resets display to "----"
          display_timeout = 0;  // cancel any pending timeout
        }
        else if (k == '#') {
          // submit
          if (entered_code == String(generatedCode)) {
            Serial.println("Code " + entered_code + " -> Correct! :)");
            setRGB(false, true, false);  // green
            dispGood();                     // "GOOD"
            display_timeout = millis() + CORRECT_DISPLAY_TIME;  // 2000ms timeout
          } else {
            Serial.println("Code " + entered_code + " -> Incorrect :(");
            setRGB(true, false, false);  // red
            dispError();                    // "XXXX"
            display_timeout = millis() + ERROR_DISPLAY_TIME;  // 1000ms timeout
          }
          // leave display showing result until next keypress or timeout
        }
        else {
          // numeric digit
          if (num_cursor >= 4) {
            Serial.println("Max 4 digits. Press # to submit or * to clear.");
            setRGB(true, false, false);
            dispError();  // "XXXX" — too many inputs
            display_timeout = millis() + ERROR_DISPLAY_TIME;  // 1000ms timeout
          } else {
            entered_code.concat(k);
            num_cursor++;
            // Serial.println("Entered so far: " + entered_code);
            setRGB(false, false, true);  // stay blue while typing
            dispCode();   // show digits entered so far, pad with "--"
            display_timeout = 0;  // cancel timeout while actively typing
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
  String displayText = current_display;
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