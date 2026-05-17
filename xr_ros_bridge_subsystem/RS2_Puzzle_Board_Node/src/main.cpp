#include <Arduino.h>
#include <button_matrix.h>
#include <led_matrix.h>
#include <game_state.h>
#include <rfid.h>

void setup(){
  Serial.begin(9600);
  generatePuzzles();
  button_matrix_setup();
  led_matrix_setup();
  rfid_setup();
}

void loop(){
  button_matrix_work();
  led_matrix_work();
  rfid_work();
}
