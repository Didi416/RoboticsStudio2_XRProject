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

// Correct Maze Definition
// 1 = lit cell (valid path), 0 = off
// Start is always (1,1), end is changeable
const bool completeMaze[8][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 1, 1, 1, 0, 0, 0, 0},
  {0, 0, 0, 1, 0, 0, 0, 0},
  {0, 0, 0, 1, 1, 1, 0, 0},
  {0, 0, 0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 0, 1, 1, 1},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
};

// End position — must match the last cell in completeMaze above
const int endX = 7;
const int endY = 5;

// RGB LED pins — adjust to match your wiring
const int RGB_RED   = 11;
const int RGB_GREEN = 10;
const int RGB_BLUE  = 9;

// ── RGB helper ───────────────────────────────────────────────

const int RGB_BRIGHTNESS = 50;  // 0–255, adjust to taste

void setRGB(bool r, bool g, bool b) {
  analogWrite(RGB_RED,   r ? RGB_BRIGHTNESS : 0);
  analogWrite(RGB_GREEN, g ? RGB_BRIGHTNESS : 0);
  analogWrite(RGB_BLUE,  b ? RGB_BRIGHTNESS : 0);
}

// ── Draw helpers ─────────────────────────────────────────────

void drawMap() {
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
}

void flashSuccess() {
  Serial.println("Correct path! :)");
  for (int i = 0; i < 4; i++) {
    matrix.clear();

    // Tick/checkmark shape on an 8x8 grid
    matrix.drawPixel(0, 4, LED_ON);
    matrix.drawPixel(1, 5, LED_ON);
    matrix.drawPixel(2, 6, LED_ON);
    matrix.drawPixel(3, 5, LED_ON);
    matrix.drawPixel(4, 4, LED_ON);
    matrix.drawPixel(5, 3, LED_ON);
    matrix.drawPixel(6, 2, LED_ON);
    matrix.drawPixel(7, 1, LED_ON);

    matrix.writeDisplay();
    setRGB(false, true, false);
    delay(300);

    matrix.clear();
    matrix.writeDisplay();
    setRGB(false, false, false);
    delay(300);
  }
  setRGB(false, true, false);
  puzzleMazeSolved = true;
}

void flashFailure() {
  Serial.println("Wrong path :(");
  for (int i = 0; i < 4; i++) {
    // Flash an X pattern
    matrix.clear();
    for (int j = 0; j < 8; j++) {
      matrix.drawPixel(j, j, LED_ON); // top-left to bottom-right
      matrix.drawPixel(7 - j, j, LED_ON); // top-right to bottom-left
    }
    matrix.writeDisplay();
    setRGB(true, false, false); // red
    delay(300);

    matrix.clear();
    matrix.writeDisplay();
    setRGB(false, false, false);
    delay(300);
  }
  setRGB(true, false, false);
}

bool validatePath() {
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
}

void led_matrix_setup() {
  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(RGB_RED,   OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLUE,  OUTPUT);

  matrix.begin(0x70);
  matrix.setBrightness(8);
  matrix.setRotation(1);

  // idleMoveTime = millis();
  resetGrid();
  drawMap();
  Serial.println("Ready. Move joystick to draw path.");
}

void led_matrix_work() {
  // Button press = clear and reset
  if (digitalRead(JOY_SW) == LOW) {
    Serial.println("Reset.");
    resetGrid();
    drawMap();
    delay(300);
    return;
  }

  // Throttle movement speed
  if (millis() - lastMove < MOVE_INTERVAL) return;

  if (idleMoveTime > 0 && (millis() - idleMoveTime) > IDLETIMER) {
    Serial.println("Timed out.");
    resetGrid();
    drawMap();
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
      drawMap();
      lastMove = millis();
      idleMoveTime = millis();

      // Check if player reached the end position
      if (posX == mazeEndX && posY == mazeEndY){
        if (validatePath()) {
          flashSuccess();
        } else {
          flashFailure();
        }
        // Pause then reset so player can try again
        delay(1000);
        resetGrid();
        drawMap();
      }
    }
  }
}

// ── Helper function for state serialization ──────────────────
String getLedMatrixState() {
  String ledMatrix = "";
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
  return ledMatrix;
}