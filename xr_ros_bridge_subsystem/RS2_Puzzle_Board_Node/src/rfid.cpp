#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <game_state.h>
#include <rfid.h>

#define RST_PIN  48
#define SS_PIN   53

MFRC522 rfid(SS_PIN, RST_PIN);

// ── Set your accepted card UID here ─────────────────────────
// Run with any card first and check Serial for its UID, then paste it here
const byte acceptedUID[4] = {0xC9, 0xFA, 0xE8, 0x6E};

// RGB pins — match your existing wiring
const int RFID_RED   = 11;
const int RFID_GREEN = 10;
const int RFID_BLUE  = 9;

void setRFIDColour(bool r, bool g, bool b) {
  digitalWrite(RFID_RED,   r);
  digitalWrite(RFID_GREEN, g);
  digitalWrite(RFID_BLUE,  b);
}

bool uidMatches(MFRC522::Uid uid) {
  if (uid.size != 4) return false;
  for (byte i = 0; i < 4; i++) {
    if (uid.uidByte[i] != acceptedUID[i]) return false;
  }
  return true;
}

void rfid_setup() {
  SPI.begin();
  rfid.PCD_Init();
  Serial.println("RFID ready.");
}

void rfid_work() {
  // No card present — nothing to do
  
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;
  // Print UID to Serial so you can find your card's UID
  // Serial.print("Card UID: ");
  // for (byte i = 0; i < rfid.uid.size; i++) {
  //   Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
  //   Serial.print(rfid.uid.uidByte[i], HEX);
  // }
  // Serial.println();

  if (!allPuzzlesSolved()) {
    Serial.println("Puzzles not complete — access denied.");
    setRFIDColour(true, false, false);   // red flash
    delay(500);
    setRFIDColour(false, false, true);   // back to blue
  }
  else if (!uidMatches(rfid.uid)) {
    Serial.println("Wrong card — access denied.");
    setRFIDColour(true, false, false);   // red flash
    delay(500);
    setRFIDColour(false, false, true);
  }
  else {
    Serial.println("Access granted!");
    // Flash green 3 times
    for (int i = 0; i < 3; i++) {
      setRFIDColour(false, true, false);
      delay(300);
      setRFIDColour(false, false, false);
      delay(300);
    }
    setRFIDColour(false, true, false);   // leave green on
  }

  rfid.PICC_HaltA();  // stop reading until card is removed and re-presented
  rfid.PCD_StopCrypto1(); //Reset reader crypto state
}