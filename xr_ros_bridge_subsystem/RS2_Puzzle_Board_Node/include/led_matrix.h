#ifndef LED_MATRIX_H_
#define LED_MATRIX_H_

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

// ── Pin config ──────────────────────────────────────────────
#define JOY_X A0
#define JOY_Y A1
#define JOY_SW 12

// ── Tuning ──────────────────────────────────────────────────
#define DEADZONE 250   // how far from 512 to ignore (reduce if sluggish, raise if drifting)
#define MOVE_INTERVAL 1000   // ms between each movement step
#define IDLETIMER 10000

// ── Matrix ──────────────────────────────────────────────────
extern Adafruit_8x8matrix matrix;

// ── State ────────────────────────────────────────────────────
extern bool currentGrid[8][8]; // visited cells
extern int posX; // current position (col), start near centre
extern int posY; // current position (row)
extern unsigned long lastMove;

void drawGrid();
void resetGrid();
void led_matrix_setup();
void led_matrix_work();

// ── Helper functions for state serialization ─────────────────
String getLedMatrixState();  // Returns 8x8 grid as "11110000,..." format

#endif /*LED_MATRIX_H*/