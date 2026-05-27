#include <Arduino.h>
#include <egg_sorter.h>
#include <game_state.h>

// ── Public state ─────────────────────────────────────────────
int  generatedEggSequence[NUM_EGGS] = {0};
bool puzzleEggSolved = false;

// ── All possible egg IDs in a fixed pool ─────────────────────
static const int EGG_POOL[NUM_EGGS] = {
    EGG_WHITE,   // 1
    EGG_GREEN,   // 2
    EGG_PURPLE,  // 3
    EGG_BLUE,    // 4
};

// ── Fisher-Yates shuffle of the egg pool ─────────────────────
// Produces a random permutation of the four egg IDs each run.
// Uses the Arduino random() which is already seeded in seedRandom()
// inside game_state.cpp before generatePuzzles() calls us.
static void shuffleEggs() {
    // Copy pool into sequence
    for (int i = 0; i < NUM_EGGS; i++) {
        generatedEggSequence[i] = EGG_POOL[i];
    }
    // Fisher-Yates
    for (int i = NUM_EGGS - 1; i > 0; i--) {
        int j = random(0, i + 1);
        int tmp = generatedEggSequence[i];
        generatedEggSequence[i] = generatedEggSequence[j];
        generatedEggSequence[j] = tmp;
    }
}

// ── Name helper ───────────────────────────────────────────────
static const char* eggName(int id) {
    switch (id) {
        case EGG_WHITE:  return "white";
        case EGG_GREEN:  return "green";
        case EGG_PURPLE: return "purple";
        case EGG_BLUE:   return "blue";
        default:         return "unknown";
    }
}

// ── Generate and print the egg sequence ──────────────────────
void generateEggOrder() {
    shuffleEggs();
    puzzleEggSolved = false;

    Serial.print("\nGenerated egg sequence (left to right): ");
    for (int i = 0; i < NUM_EGGS; i++) {
        Serial.print(generatedEggSequence[i]);
        Serial.print("(");
        Serial.print(eggName(generatedEggSequence[i]));
        Serial.print(")");
        if (i < NUM_EGGS - 1) Serial.print(", ");
    }
    Serial.println();
}

// ── Check incoming serial for egg solved confirmation ─────────
// The Python verifier sends "EGG_SOLVED\n" over serial when the
// detected X-order matches the expected sequence.
// This is called from egg_sorter_work() every loop tick.
static void checkSerialForSolved() {
    if (Serial.available() > 0) {
        String incoming = Serial.readStringUntil('\n');
        incoming.trim();
        if (incoming == "EGG_SOLVED") {
            puzzleEggSolved = true;
            Serial.println("Egg puzzle confirmed solved by vision system.");
        }
    }
}

// ── Setup / loop ──────────────────────────────────────────────

void egg_sorter_setup() {
    // Nothing hardware-specific needed for the egg sorter —
    // detection is done by the ROS vision system, not the Arduino.
    // Generation happens inside generatePuzzles() in game_state.cpp.
}

void egg_sorter_work() {
    // Poll serial for EGG_SOLVED confirmation from the Python verifier.
    // Only check if not already solved this run.
    if (!puzzleEggSolved) {
        checkSerialForSolved();
    }
}

// ── Serialise sequence for sendPuzzleGeneration() in main.cpp ─
// Returns format: "EGG:1,3,2,4"  (space-separated ArUco IDs in order)
String getEggSequence() {
    String s = "";
    for (int i = 0; i < NUM_EGGS; i++) {
        s += String(generatedEggSequence[i]);
        if (i < NUM_EGGS - 1) s += ",";
    }
    return s;
}