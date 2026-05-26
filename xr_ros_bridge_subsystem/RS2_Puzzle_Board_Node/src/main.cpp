// #include <Arduino.h>
// #include <button_matrix.h>
// #include <led_matrix.h>
// #include <game_state.h>
// #include <rfid.h>

// int joystick_switch = 12;
// int joyX_state = 0;
// int joyY_state = 0;

// String display_state = "";

// void updateState(){
//   // Collect joystick analog values and switch state
//   int joyX = analogRead(A0);
//   int joyY = analogRead(A1);
//   int joySw = digitalRead(joystick_switch) == LOW ? 1 : 0;
  
//   // Collect button matrix states
//   String buttonStates = getButtonStates();
  
//   // Collect LED matrix state (8x8 grid as binary strings)
//   String ledMatrix = getLedMatrixState();
  
//   // Collect alphanumeric display state
//   String displayText = getDisplayText();
  
//   // Collect puzzle state (1 if all solved, 0 otherwise)
//   int puzzleState = allPuzzlesSolved() ? 1 : 0;
  
//   // Build the state string in the specified format
//   String stateStr = "J:" + String(joyX) + "," + String(joyY) + "," + String(joySw);
//   // stateStr += "|B:" + buttonStates;
//   stateStr += "|L:" + ledMatrix;
//   stateStr += "|T:" + displayText;
//   stateStr += "|P:" + String(puzzleState);
  
//   // Send the state via Serial
//   Serial.println(stateStr);
// }

// void sendPuzzleGeneration(){
//   // Send the initial puzzle generation solutions in similar format
//   // Format: CODE:<code>|MAZE:<8x8_binary_grid>|MAZE_END:x,y
  
//   String mazeStr = "";
//   for (int row = 0; row < 8; row++) {
//     for (int col = 0; col < 8; col++) {
//       if (generatedMaze[row][col]) {
//         mazeStr += "1";
//       } else {
//         mazeStr += "0";
//       }
//     }
//     if (row < 7) mazeStr += ",";
//   }
  
//   String puzzleGenStr = "CODE:" + String(generatedCode);
//   puzzleGenStr += "|MAZE:" + mazeStr;
//   puzzleGenStr += "|MAZE_END:" + String(mazeEndX) + "," + String(mazeEndY);
  
//   Serial.println(puzzleGenStr);
// }

// void setup(){
//   Serial.begin(9600);
//   generatePuzzles();
//   sendPuzzleGeneration();
//   button_matrix_setup();
//   led_matrix_setup();
//   rfid_setup();
//   pinMode(joystick_switch, INPUT_PULLUP);
// }

// void loop(){
//   if (digitalRead(joystick_switch) == LOW) {
//     Serial.println("RESET BOARD");
//     resetPuzzles();
//     sendPuzzleGeneration();
//     delay(500);
//   }
//   button_matrix_work();
//   led_matrix_work();
//   rfid_work();
//   updateState();
//   delay(100);  // Send state update every 100ms
// }

#include <Arduino.h>
#include <button_matrix.h>
#include <led_matrix.h>
#include <game_state.h>
#include <rfid.h>

int joystick_switch = 12;
int joyX_state = 0;
int joyY_state = 0;
String display_state = "";

// Timer for puzzle generation sending (5 seconds = 5000ms)
unsigned long lastPuzzleGenSendTime = 0;
const unsigned long PUZZLE_GEN_INTERVAL = 5000;

void updateState(){
  // Collect joystick analog values and switch state
  int joyX = analogRead(A0);
  int joyY = analogRead(A1);
  int joySw = digitalRead(joystick_switch) == LOW ? 1 : 0;
  
  // Collect button matrix states
  String buttonStates = getButtonStates();
  
  // Collect LED matrix state (8x8 grid as binary strings)
  String ledMatrix = getLedMatrixState();
  
  // Collect alphanumeric display state
  String displayText = getDisplayText();
  
  // Collect puzzle state (1 if all solved, 0 otherwise)
  int puzzleState = allPuzzlesSolved() ? 1 : 0;
  
  // Build the state string in the specified format
  String stateStr = "J:" + String(joyX) + "," + String(joyY) + "," + String(joySw);
  // stateStr += "|B:" + buttonStates;
  stateStr += "|L:" + ledMatrix;
  stateStr += "|T:" + displayText;
  stateStr += "|P:" + String(puzzleState);
  
  // Send the state via Serial
  Serial.println(stateStr);
}

void sendPuzzleGeneration(){
  // Send the initial puzzle generation solutions in similar format
  // Format: CODE:<code>|MAZE:<8x8_binary_grid>|MAZE_END:x,y
  String mazeStr = "";
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      if (generatedMaze[row][col]) {
        mazeStr += "1";
      } else {
        mazeStr += "0";
      }
    }
    if (row < 7) mazeStr += ",";
  }
  
  String puzzleGenStr = "CODE:" + String(generatedCode);
  puzzleGenStr += "|MAZE:" + mazeStr;
  puzzleGenStr += "|MAZE_END:" + String(mazeEndX) + "," + String(mazeEndY);
  Serial.println(puzzleGenStr);
}

void setup(){
  Serial.begin(9600);
  generatePuzzles();
  sendPuzzleGeneration();
  button_matrix_setup();
  led_matrix_setup();
  rfid_setup();
  pinMode(joystick_switch, INPUT_PULLUP);
  
  // Initialize timer
  lastPuzzleGenSendTime = millis();
}

void loop(){
  unsigned long currentTime = millis();
  
  if (digitalRead(joystick_switch) == LOW) {
    Serial.println("RESET BOARD");
    resetPuzzles();
    sendPuzzleGeneration();
    lastPuzzleGenSendTime = currentTime;  // Reset timer on manual reset
    delay(500);
  }
  
  // Send puzzle generation every 5 seconds
  if (currentTime - lastPuzzleGenSendTime >= PUZZLE_GEN_INTERVAL) {
    sendPuzzleGeneration();
    lastPuzzleGenSendTime = currentTime;
  }
  
  button_matrix_work();
  led_matrix_work();
  rfid_work();
  updateState();
  
  delay(100);  // Send state update every 100ms
}