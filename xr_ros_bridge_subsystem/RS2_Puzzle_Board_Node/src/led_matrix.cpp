#include <Arduino.h>
#include <led_matrix.h>
#include <game_state.h>

Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
bool currentGrid[8][8] = {};
int posX = 1;
int posY = 1;
unsigned long lastMove = 0;
unsigned long idleMoveTime = 0;
float dimLEDs = 0.5;

// Timeout states for flash animations
unsigned long flashTimeout = 0;
unsigned long resetTimeout = 0;
unsigned long buttonDebounceTimeout = 0;

int flashState = -1;  // -1 = not flashing, 0+ = flash frame number
bool isFlashingSuccess = false;  // Track which type of flash is active

const unsigned long FLASH_INTERVAL = 300;      // 300ms per flash frame
const unsigned long POST_FLASH_RESET = 1000;   // 1000ms delay after reaching end
const unsigned long BUTTON_DEBOUNCE = 300;     // 300ms debounce for button

// Correct Maze Definition
// 1 = lit cell (valid path), 0 = off
// Start is always (1,1), end is changeable
const bool success[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 1},
  {0, 0, 0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1, 0, 0},
  {1, 0, 0, 0, 1, 0, 0, 0},
  {0, 1, 0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
};
const bool fail[8][8] = {
  {1, 0, 0, 0, 0, 0, 0, 1},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {0, 0, 1, 0, 0, 1, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 0, 1, 1, 0, 0, 0},
  {0, 0, 1, 0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {1, 0, 0, 0, 0, 0, 0, 1},
};

bool failed = false;
bool succeeded = false;

// End position — must match the last cell in completeMaze above
const int endX = 7;
const int endY = 5;

// ── Draw helpers ─────────────────────────────────────────────

void drawGrid() {
  matrix.clear();
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      if (currentGrid[row][col]) {
        matrix.drawPixel(col, row, LED_ON);
      }
    }
  }
  matrix.drawPixel(posX, posY, LED_ON);  // current position always visible
  matrix.writeDisplay();
  // store_constant = currentGrid;
}

void drawSuccess() {
  matrix.clear();
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      if (success[row][col]) {
        matrix.drawPixel(col, row, LED_ON);
      }
    }
  }
  matrix.drawPixel(posX, posY, LED_ON);  // current position always visible
  matrix.writeDisplay();
}

void drawFailure() {
  matrix.clear();
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      if (fail[row][col]) {
        matrix.drawPixel(col, row, LED_ON);
      }
    }
  }
  matrix.drawPixel(posX, posY, LED_ON);  // current position always visible
  matrix.writeDisplay();
}

void startFlashSuccess() {
  Serial.println("Correct path! :)");
  flashState = 0;
  isFlashingSuccess = true;
  flashTimeout = millis() + FLASH_INTERVAL;
  setRGB(false, true, false);
}

void updateFlashSuccess() {
  unsigned long now = millis();
  
  if (now >= flashTimeout) {
    if (flashState % 2 == 0) {
      // Flash on - draw checkmark
      // matrix.clear();
      // matrix.drawPixel(0, 4, LED_ON);
      // matrix.drawPixel(1, 5, LED_ON);
      // matrix.drawPixel(2, 6, LED_ON);
      // matrix.drawPixel(3, 5, LED_ON);
      // matrix.drawPixel(4, 4, LED_ON);
      // matrix.drawPixel(5, 3, LED_ON);
      // matrix.drawPixel(6, 2, LED_ON);
      // matrix.drawPixel(7, 1, LED_ON);
      // matrix.writeDisplay();
      drawSuccess();
      setRGB(false, true, false);
    } else {
      // Flash off
      matrix.clear();
      matrix.writeDisplay();
      setRGB(false, false, false);
    }
    
    flashState++;
    flashTimeout = now + FLASH_INTERVAL;
    
    // After 4 frames (8 states), finish flashing
    if (flashState >= 8) {
      flashState = -1;
      isFlashingSuccess = false;
      setRGB(false, true, false);
      puzzleMazeSolved = true;
      resetTimeout = now + POST_FLASH_RESET;
      succeeded = false;
    }
  }
}

void startFlashFailure() {
  Serial.println("Wrong path :(");
  flashState = 0;
  isFlashingSuccess = false;
  flashTimeout = millis() + FLASH_INTERVAL;
  setRGB(true, false, false);
}

void updateFlashFailure() {
  unsigned long now = millis();
  
  if (now >= flashTimeout) {
    if (flashState % 2 == 0) {
      // Flash on - draw X pattern
      // matrix.clear();
      // for (int j = 0; j < 8; j++) {
      //   matrix.drawPixel(j, j, LED_ON);           // top-left to bottom-right
      //   matrix.drawPixel(7 - j, j, LED_ON);       // top-right to bottom-left
      // }
      // matrix.writeDisplay();
      drawFailure();
      setRGB(true, false, false);
    } else {
      // Flash off
      matrix.clear();
      matrix.writeDisplay();
      setRGB(false, false, false);
    }
    
    flashState++;
    flashTimeout = now + FLASH_INTERVAL;
    
    // After 4 frames (8 states), finish flashing
    if (flashState >= 8) {
      flashState = -1;
      isFlashingSuccess = false;
      setRGB(true, false, false);
      resetTimeout = now + POST_FLASH_RESET;
      failed = false;
    }
  }
}

bool validatePath() {
  Serial.println("Validating...");
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      if (currentGrid[row][col] != generatedMaze[row][col]) {
        return false;
      }
    }
  }
  return true;
}

void resetGrid() {
  memset(currentGrid, 0, sizeof(currentGrid));
  posX = 1;
  posY = 1;
  currentGrid[posY][posX] = true;
  setRGB(false, false, true);   // back to blue while playing
  flashState = -1;
  isFlashingSuccess = false;
  flashTimeout = 0;
  resetTimeout = 0;
}

void led_matrix_setup() {
  pinMode(JOY_SW, INPUT_PULLUP);

  matrix.begin(0x70);
  matrix.setBrightness(8);
  matrix.setRotation(1);

  resetGrid();
  drawGrid();
  Serial.println("Ready. Move joystick to draw path.");
}

void led_matrix_work() {
  unsigned long now = millis();

  // Handle button press with debounce
  if (digitalRead(JOY_SW) == LOW && now >= buttonDebounceTimeout) {
    Serial.println("Reset.");
    resetGrid();
    drawGrid();
    buttonDebounceTimeout = now + BUTTON_DEBOUNCE;
    return;
  }

  // Handle reset timeout after reaching end
  if (resetTimeout > 0 && now >= resetTimeout) {
    resetTimeout = 0;
    resetGrid();
    drawGrid();
    return;
  }

  // Update flashing animations
  if (flashState >= 0) {
    if (isFlashingSuccess) {
      updateFlashSuccess();
      succeeded = true;
    } else {
      updateFlashFailure();
      failed = true;
    }
    return;  // Don't process movement while flashing
  }
  failed = false;
  succeeded = false;

  // Throttle movement speed
  if (now - lastMove < MOVE_INTERVAL) return;

  if (idleMoveTime > 0 && (now - idleMoveTime) > IDLETIMER) {
    Serial.println("Timed out.");
    resetGrid();
    drawGrid();
    idleMoveTime = 0;
    return;
  }

  int rawX = analogRead(JOY_X);
  int rawY = analogRead(JOY_Y);

  int dx = 0;
  int dy = 0;

  if (rawX < 512 - DEADZONE)      dx = -1;
  else if (rawX > 512 + DEADZONE) dx = 1;

  if (rawY < 512 - DEADZONE)      dy = 1;
  else if (rawY > 512 + DEADZONE) dy = -1;

  // if (rawX < 512 - DEADZONE)      dy = -1;
  // else if (rawX > 512 + DEADZONE) dy = 1;
  // if (rawY < 512 - DEADZONE)      dx = -1;
  // else if (rawY > 512 + DEADZONE) dx = 1;

  if ((dx != 0) != (dy != 0)) {
    int newX = constrain(posX + dx, 0, 7);
    int newY = constrain(posY + dy, 0, 7);
    
    if (newX != posX || newY != posY) {
      posX = newX;
      posY = newY;
      currentGrid[posY][posX] = true;
      drawGrid();
      lastMove = now;
      idleMoveTime = now;
      
      // Check if player reached the end position
      if (posX == mazeEndX && posY == mazeEndY) {
        // Serial.println("Test end");
        if (validatePath()) {
          startFlashSuccess();
        } else {
          startFlashFailure();
        }
        // Will reset automatically after POST_FLASH_RESET via resetTimeout
      }
      else {
        // Serial.print(posX);
        // Serial.print(posY);
        // Serial.print(mazeEndX);
        // Serial.println(mazeEndY);
      }
    }
  }
}

// ── Helper function for state serialization ──────────────────
String getLedMatrixState() {
  String ledMatrix = "";
  if(succeeded){
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        if (success[row][col]) {
          ledMatrix += "1";
        } else {
          ledMatrix += "0";
        }
      }
      if (row < 7) ledMatrix += ",";
    }
  } else if (failed){
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        if (fail[row][col]) {
          ledMatrix += "1";
        } else {
          ledMatrix += "0";
        }
      }
      if (row < 7) ledMatrix += ",";
    }
  } else {
    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        if (currentGrid[row][col]) {
          ledMatrix += "1";
        } else {
          ledMatrix += "0";
        }
      }
      if (row < 7) ledMatrix += ",";
    }
  }
  return ledMatrix;
}

// #include <Arduino.h>
// #include <led_matrix.h>
// #include <game_state.h>

// Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
// bool currentGrid[8][8] = {};
// int posX = 1;
// int posY = 1;
// unsigned long lastMove = 0;
// unsigned long idleMoveTime = 0;
// float dimLEDs = 0.5;

// // Correct Maze Definition
// // 1 = lit cell (valid path), 0 = off
// // Start is always (1,1), end is changeable
// const bool completeMaze[8][8] = {
//   {0, 0, 0, 0, 0, 0, 0, 0},
//   {0, 1, 1, 1, 0, 0, 0, 0},
//   {0, 0, 0, 1, 0, 0, 0, 0},
//   {0, 0, 0, 1, 1, 1, 0, 0},
//   {0, 0, 0, 0, 0, 1, 0, 0},
//   {0, 0, 0, 0, 0, 1, 1, 1},
//   {0, 0, 0, 0, 0, 0, 0, 0},
//   {0, 0, 0, 0, 0, 0, 0, 0},
// };

// // End position — must match the last cell in completeMaze above
// const int endX = 7;
// const int endY = 5;

// // ── Draw helpers ─────────────────────────────────────────────

// void drawGrid() {
//   matrix.clear();
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (currentGrid[row][col]) {
//         matrix.drawPixel(col, row, LED_ON);
//       }
//     }
//   }
//   matrix.drawPixel(posX, posY, LED_ON);  // current position always visible
//   matrix.writeDisplay();
// }

// void flashSuccess() {
//   Serial.println("Correct path! :)");
//   for (int i = 0; i < 4; i++) {
//     matrix.clear();

//     // Tick/checkmark shape on an 8x8 grid
//     matrix.drawPixel(0, 4, LED_ON);
//     matrix.drawPixel(1, 5, LED_ON);
//     matrix.drawPixel(2, 6, LED_ON);
//     matrix.drawPixel(3, 5, LED_ON);
//     matrix.drawPixel(4, 4, LED_ON);
//     matrix.drawPixel(5, 3, LED_ON);
//     matrix.drawPixel(6, 2, LED_ON);
//     matrix.drawPixel(7, 1, LED_ON);

//     matrix.writeDisplay();
//     setRGB(false, true, false);
//     delay(300);

//     matrix.clear();
//     matrix.writeDisplay();
//     setRGB(false, false, false);
//     delay(300);
//   }
//   setRGB(false, true, false);
//   puzzleMazeSolved = true;
// }

// void flashFailure() {
//   Serial.println("Wrong path :(");
//   for (int i = 0; i < 4; i++) {
//     // Flash an X pattern
//     matrix.clear();
//     for (int j = 0; j < 8; j++) {
//       matrix.drawPixel(j, j, LED_ON); // top-left to bottom-right
//       matrix.drawPixel(7 - j, j, LED_ON); // top-right to bottom-left
//     }
//     matrix.writeDisplay();
//     setRGB(true, false, false); // red
//     delay(300);

//     matrix.clear();
//     matrix.writeDisplay();
//     setRGB(false, false, false);
//     delay(300);
//   }
//   setRGB(true, false, false);
// }

// bool validatePath() {
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (currentGrid[row][col] != generatedMaze[row][col]) {
//         return false;
//       }
//     }
//   }
//   return true;
// }

// void resetGrid() {
//   memset(currentGrid, 0, sizeof(currentGrid));
//   posX = 1;
//   posY = 1;
//   currentGrid[posY][posX] = true;
//   setRGB(false, false, true);   // back to blue while playing
// }

// void led_matrix_setup() {
//   pinMode(JOY_SW, INPUT_PULLUP);

//   matrix.begin(0x70);
//   matrix.setBrightness(8);
//   matrix.setRotation(1);

//   // idleMoveTime = millis();
//   resetGrid();
//   drawGrid();
//   Serial.println("Ready. Move joystick to draw path.");
// }

// void led_matrix_work() {
//   // Button press = clear and reset
//   if (digitalRead(JOY_SW) == LOW) {
//     Serial.println("Reset.");
//     resetGrid();
//     drawGrid();
//     delay(300);
//     return;
//   }

//   // Throttle movement speed
//   if (millis() - lastMove < MOVE_INTERVAL) return;

//   if (idleMoveTime > 0 && (millis() - idleMoveTime) > IDLETIMER) {
//     Serial.println("Timed out.");
//     resetGrid();
//     drawGrid();
//     idleMoveTime = 0;
//     return;
//   }

//   int rawX = analogRead(JOY_X);
//   int rawY = analogRead(JOY_Y);

//   int dx = 0;
//   int dy = 0;

//   if (rawX < 512 - DEADZONE)      dx = 1;
//   else if (rawX > 512 + DEADZONE) dx = -1;

//   if (rawY < 512 - DEADZONE)      dy = -1;
//   else if (rawY > 512 + DEADZONE) dy = 1;

//   // if (rawX < 512 - DEADZONE)      dy = -1;
//   // else if (rawX > 512 + DEADZONE) dy = 1;
//   // if (rawY < 512 - DEADZONE)      dx = -1;
//   // else if (rawY > 512 + DEADZONE) dx = 1;

//   if ((dx != 0) != (dy != 0)) {
//     int newX = constrain(posX + dx, 0, 7);
//     int newY = constrain(posY + dy, 0, 7);

//     if (newX != posX || newY != posY) {
//       posX = newX;
//       posY = newY;
//       currentGrid[posY][posX] = true;
//       drawGrid();
//       lastMove = millis();
//       idleMoveTime = millis();

//       // Check if player reached the end position
//       if (posX == mazeEndX && posY == mazeEndY){
//         if (validatePath()) {
//           flashSuccess();
//         } else {
//           flashFailure();
//         }
//         // Pause then reset so player can try again
//         delay(1000);
//         resetGrid();
//         drawGrid();
//       }
//     }
//   }
// }

// // ── Helper function for state serialization ──────────────────
// String getLedMatrixState() {
//   String ledMatrix = "";
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (currentGrid[row][col]) {
//         ledMatrix += "1";
//       } else {
//         ledMatrix += "0";
//       }
//     }
//     if (row < 7) ledMatrix += ",";
//   }
//   return ledMatrix;
// }

// #include <Arduino.h>
// #include <led_matrix.h>
// #include <game_state.h>

// Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
// bool currentGrid[8][8] = {};
// int posX = 1;
// int posY = 1;
// unsigned long lastMove = 0;
// unsigned long idleMoveTime = 0;
// float dimLEDs = 0.5;

// // Timeout states
// unsigned long flashTimeout = 0;
// unsigned long resetTimeout = 0;
// unsigned long buttonDebounceTimeout = 0;

// int flashState = -1;  // -1 = not flashing, 0+ = flash frame number
// bool isFlashingSuccess = false;  // Track which type of flash is active

// const unsigned long FLASH_INTERVAL = 300;      // 300ms per flash frame
// const unsigned long FLASH_TOTAL_TIME = 2400;   // 4 frames × 300ms = 2400ms
// const unsigned long RESET_DELAY = 1000;        // 1000ms delay after reaching end
// const unsigned long BUTTON_DEBOUNCE = 300;     // 300ms debounce for button

// // Correct Maze Definition
// // 1 = lit cell (valid path), 0 = off
// // Start is always (1,1), end is changeable
// const bool completeMaze[8][8] = {
//   {0, 0, 0, 0, 0, 0, 0, 0},
//   {0, 1, 1, 1, 0, 0, 0, 0},
//   {0, 0, 0, 1, 0, 0, 0, 0},
//   {0, 0, 0, 1, 1, 1, 0, 0},
//   {0, 0, 0, 0, 0, 1, 0, 0},
//   {0, 0, 0, 0, 0, 1, 1, 1},
//   {0, 0, 0, 0, 0, 0, 0, 0},
//   {0, 0, 0, 0, 0, 0, 0, 0},
// };

// // End position — must match the last cell in completeMaze above
// const int endX = 7;
// const int endY = 5;

// // ── Draw helpers ─────────────────────────────────────────────

// void drawGrid() {
//   matrix.clear();
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (currentGrid[row][col]) {
//         matrix.drawPixel(col, row, LED_ON);
//       }
//     }
//   }
//   matrix.drawPixel(posX, posY, LED_ON);  // current position always visible
//   matrix.writeDisplay();
// }

// void startFlashSuccess() {
//   Serial.println("Correct path! :)");
//   flashState = 0;
//   isFlashingSuccess = true;
//   flashTimeout = millis() + FLASH_INTERVAL;
//   setRGB(false, true, false);
// }

// void updateFlashSuccess() {
//   unsigned long now = millis();
  
//   if (now >= flashTimeout) {
//     if (flashState % 2 == 0) {
//       // Flash on - draw checkmark
//       matrix.clear();
//       matrix.drawPixel(0, 4, LED_ON);
//       matrix.drawPixel(1, 5, LED_ON);
//       matrix.drawPixel(2, 6, LED_ON);
//       matrix.drawPixel(3, 5, LED_ON);
//       matrix.drawPixel(4, 4, LED_ON);
//       matrix.drawPixel(5, 3, LED_ON);
//       matrix.drawPixel(6, 2, LED_ON);
//       matrix.drawPixel(7, 1, LED_ON);
//       matrix.writeDisplay();
//       setRGB(false, true, false);
//     } else {
//       // Flash off
//       matrix.clear();
//       matrix.writeDisplay();
//       setRGB(false, false, false);
//     }
    
//     flashState++;
//     flashTimeout = now + FLASH_INTERVAL;
    
//     // After 4 frames (8 states), finish flashing
//     if (flashState >= 8) {
//       flashState = -1;
//       isFlashingSuccess = false;
//       setRGB(false, true, false);
//       puzzleMazeSolved = true;
//       resetTimeout = now + RESET_DELAY;
//     }
//   }
// }

// void startFlashFailure() {
//   Serial.println("Wrong path :(");
//   flashState = 0;
//   isFlashingSuccess = false;
//   flashTimeout = millis() + FLASH_INTERVAL;
//   setRGB(true, false, false);
// }

// void updateFlashFailure() {
//   unsigned long now = millis();
  
//   if (now >= flashTimeout) {
//     if (flashState % 2 == 0) {
//       // Flash on - draw X pattern
//       matrix.clear();
//       for (int j = 0; j < 8; j++) {
//         matrix.drawPixel(j, j, LED_ON);           // top-left to bottom-right
//         matrix.drawPixel(7 - j, j, LED_ON);       // top-right to bottom-left
//       }
//       matrix.writeDisplay();
//       setRGB(true, false, false);
//     } else {
//       // Flash off
//       matrix.clear();
//       matrix.writeDisplay();
//       setRGB(false, false, false);
//     }
    
//     flashState++;
//     flashTimeout = now + FLASH_INTERVAL;
    
//     // After 4 frames (8 states), finish flashing
//     if (flashState >= 8) {
//       flashState = -1;
//       isFlashingSuccess = false;
//       setRGB(true, false, false);
//       resetTimeout = now + RESET_DELAY;
//     }
//   }
// }

// bool validatePath() {
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (currentGrid[row][col] != generatedMaze[row][col]) {
//         return false;
//       }
//     }
//   }
//   return true;
// }

// void resetGrid() {
//   memset(currentGrid, 0, sizeof(currentGrid));
//   posX = 1;
//   posY = 1;
//   currentGrid[posY][posX] = true;
//   setRGB(false, false, true);   // back to blue while playing
//   flashState = -1;
//   isFlashingSuccess = false;
//   flashTimeout = 0;
//   resetTimeout = 0;
// }

// void led_matrix_setup() {
//   pinMode(JOY_SW, INPUT_PULLUP);
//   matrix.begin(0x70);
//   matrix.setBrightness(8);
//   matrix.setRotation(1);

//   resetGrid();
//   drawGrid();
//   Serial.println("Ready. Move joystick to draw path.");
// }

// void led_matrix_work() {
//   unsigned long now = millis();

//   // Handle button press with debounce
//   if (digitalRead(JOY_SW) == LOW && now >= buttonDebounceTimeout) {
//     Serial.println("Reset.");
//     resetGrid();
//     drawGrid();
//     buttonDebounceTimeout = now + BUTTON_DEBOUNCE;
//     return;
//   }

//   // Handle reset timeout after reaching end
//   if (resetTimeout > 0 && now >= resetTimeout) {
//     resetTimeout = 0;
//     resetGrid();
//     drawGrid();
//     return;
//   }

//   // Update flashing animations
//   if (flashState >= 0) {
//     if (isFlashingSuccess) {
//       updateFlashSuccess();
//     } else {
//       updateFlashFailure();
//     }
//     return;  // Don't process movement while flashing
//   }

//   // Throttle movement speed
//   if (now - lastMove < MOVE_INTERVAL) return;

//   if (idleMoveTime > 0 && (now - idleMoveTime) > IDLETIMER) {
//     Serial.println("Timed out.");
//     resetGrid();
//     drawGrid();
//     idleMoveTime = 0;
//     return;
//   }

//   int rawX = analogRead(JOY_X);
//   int rawY = analogRead(JOY_Y);

//   int dx = 0;
//   int dy = 0;

//   if (rawX < 512 - DEADZONE)      dx = 1;
//   else if (rawX > 512 + DEADZONE) dx = -1;

//   if (rawY < 512 - DEADZONE)      dy = -1;
//   else if (rawY > 512 + DEADZONE) dy = 1;

//   if ((dx != 0) != (dy != 0)) {
//     int newX = constrain(posX + dx, 0, 7);
//     int newY = constrain(posY + dy, 0, 7);

//     if (newX != posX || newY != posY) {
//       posX = newX;
//       posY = newY;
//       currentGrid[posY][posX] = true;
//       drawGrid();
//       lastMove = now;
//       idleMoveTime = now;

//       // Check if player reached the end position
//       if (posX == endX && posY == endY) {
//         if (validatePath()) {
//           startFlashSuccess();
//         } else {
//           startFlashFailure();
//         }
//         // Will reset automatically after RESET_DELAY via resetTimeout
//       }
//     }
//   }
// }

// // ── Helper function for state serialization ──────────────────
// String getLedMatrixState() {
//   String ledMatrix = "";
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (currentGrid[row][col]) {
//         ledMatrix += "1";
//       } else {
//         ledMatrix += "0";
//       }
//     }
//     if (row < 7) ledMatrix += ",";
//   }
//   return ledMatrix;
// }