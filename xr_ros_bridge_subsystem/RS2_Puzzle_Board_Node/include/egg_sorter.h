#pragma once
#include <Arduino.h>  

// ── Egg ArUco IDs ────────────────────────────────────────────
// These must match the IDs printed on your physical egg blocks.
// IDs 0,1,2,3,10 are already used by other puzzle markers — do not reuse them.
#define EGG_WHITE  1
#define EGG_GREEN  2
#define EGG_PURPLE 3
#define EGG_BLUE   4

#define NUM_EGGS 4

// ── Public state (readable from game_state.cpp and main.cpp) ─
extern int  generatedEggSequence[NUM_EGGS];   // correct left-to-right order
extern bool puzzleEggSolved;                  // set true when Python verifier confirms

// ── Public functions ─────────────────────────────────────────
void egg_sorter_setup();
void egg_sorter_work();
String getEggSequence();    // for sendPuzzleGeneration() in main.cpp
void generateEggOrder();   // game_state.cpp calls this