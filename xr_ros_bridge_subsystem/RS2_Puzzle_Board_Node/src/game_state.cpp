#include <Arduino.h>
#include <game_state.h>
#include <egg_sorter.h>

bool puzzleMazeSolved = false;
bool puzzleCodeSolved = false;
// puzzleEggSolved lives in egg_sorter.cpp — extern declared in egg_sorter.h

char generatedCode[5] = {0}; //null terminated 4 digit code
bool generatedMaze[8][8] = {}; 
int mazeEndX = 1;
int mazeEndY = 1;

bool allPuzzlesSolved() {
  return puzzleMazeSolved && puzzleCodeSolved && puzzleEggSolved;
}

static void seedRandom() {
  long seed = 0;
  for (int i = 0; i < 8; i++) {
    seed ^= (long)analogRead(A5) << (i * 4);
    delay(2);
  }
  randomSeed(seed);
}

// ── Code generator ───────────────────────────────────────────
static void generateCode() {
  for (int i = 0; i < 4; i++) {
    generatedCode[i] = '0' + random(1, 9);  // digits 1–9, avoids 0 for clarity
  }
  generatedCode[4] = '\0';

  Serial.print("\nGenerated code: ");
  Serial.println(generatedCode);
}

// ── Maze generator ───────────────────────────────────────────
// Carves a random walk from (1,1), never revisiting a cell,
// stopping when stuck or after hitting a max path length.
// Direction order is shuffled each step so the path isn't biased.

static const int dx[4] = { 0,  0, -1,  1 };  // up, down, left, right
static const int dy[4] = {-1,  1,  0,  0 };

static void shuffleDirs(int dirs[4]) {
  for (int i = 3; i > 0; i--) {
    int j = random(0, i + 1);
    int tmp = dirs[i];
    dirs[i] = dirs[j];
    dirs[j] = tmp;
  }
}

static void generateMaze() {
  memset(generatedMaze, 0, sizeof(generatedMaze));

  int x = 1;
  int y = 1;
  generatedMaze[y][x] = true;

  // Path length: between 6 and 12 steps
  int targetLen = random(6, 13);
  int steps = 0;

  for (int attempt = 0; attempt < 200 && steps < targetLen; attempt++) {
    int dirs[4] = {0, 1, 2, 3};
    shuffleDirs(dirs);

    bool moved = false;
    for (int d = 0; d < 4; d++) {
      int nx = x + dx[dirs[d]];
      int ny = y + dy[dirs[d]];

      // Stay within inner bounds — keep a 1-cell border clear for readability
      if (nx < 1 || nx > 6 || ny < 1 || ny > 6) continue;

      // Don't revisit
      if (generatedMaze[ny][nx]) continue;

      // Don't allow diagonal connectivity —
      // check that moving here doesn't touch another visited cell
      // at a corner (would create an ambiguous path)
      bool tooClose = false;
      for (int dd = 0; dd < 4; dd++) {
        int cx = nx + dx[dd];
        int cy = ny + dy[dd];
        if (cx == x && cy == y) continue;  // that's where we came from, fine
        if (cx < 0 || cx > 7 || cy < 0 || cy > 7) continue;
        if (generatedMaze[cy][cx]) { tooClose = true; break; }
      }
      if (tooClose) continue;

      x = nx;
      y = ny;
      generatedMaze[y][x] = true;
      steps++;
      moved = true;
      break;
    }

    // If no valid move found, path is stuck — stop here
    if (!moved) break;
  }

  mazeEndX = x;
  mazeEndY = y;
}

void printMaze() {
  Serial.println("Generated maze (X = path, _ = empty):");
  Serial.println("  01234567");
  for (int row = 0; row < 8; row++) {
    Serial.print(row);
    Serial.print(" ");
    for (int col = 0; col < 8; col++) {
      if (col == 1 && row == 1)       Serial.print("S");  // start
      else if (col == mazeEndX && row == mazeEndY) Serial.print("E");  // end
      else if (generatedMaze[row][col]) Serial.print("X");
      else Serial.print("_");
    }
    Serial.println();
  }
  Serial.print("Start: (1,1)  End: (");
  Serial.print(mazeEndX);
  Serial.print(",");
  Serial.print(mazeEndY);
  Serial.println(")");
}

// ── Public entry point ───────────────────────────────────────
void generatePuzzles() {
  seedRandom();
  generateCode();
  generateMaze();
  printMaze();
  generateEggOrder();  // generate egg order after maze (same seed)
}

void resetPuzzles() {
  puzzleMazeSolved = false;
  puzzleCodeSolved = false;
  // puzzleEggSolved is reset inside generateEggSequence()
  generatePuzzles();
}