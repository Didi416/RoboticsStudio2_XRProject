#ifndef BUTTON_MATRIX_H_
#define BUTTON_MATRIX_H_

#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

#define ROWS 4
#define COLS 3

//setup to define elsewhere
extern char keys[ROWS][COLS];
extern byte rowPins[ROWS];
extern byte colPins[COLS];
extern Keypad kpd;
extern Adafruit_AlphaNum4 disp;

extern String correct_code;
extern String entered_code;
extern int num_cursor;

extern int redPin;
extern int greenPin;
extern int bluePin;

//Funtion defintions
void setColour(bool red, bool green, bool blue);
void resetEntry();
void button_matrix_setup();
void button_matrix_work();

#endif /*BUTTON_MATRIX_H*/