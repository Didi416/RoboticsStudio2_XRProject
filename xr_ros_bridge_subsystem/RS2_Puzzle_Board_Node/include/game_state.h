#ifndef GAME_STATE_H
#define GAME_STATE_H

#include <Arduino.h>

extern bool puzzleMazeSolved;
extern bool puzzleCodeSolved;

// ── Shared generated puzzles ─────────────────────────────────
extern char generatedCode[5];      // 4 digits + null terminator
extern bool generatedMaze[8][8];   // the correct maze path
extern int mazeEndX;
extern int mazeEndY;

bool allPuzzlesSolved();
void generatePuzzles();
void resetPuzzles();
void updateState();
void setRGB(bool r, bool g, bool b);

#endif