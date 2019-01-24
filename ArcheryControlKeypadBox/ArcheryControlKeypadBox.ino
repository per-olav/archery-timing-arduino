#include <arduino.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <string.h>

#define NODE_ID          1
#define OUTPUT_PIN      12

#define LED_PIN         10
#define MAX_KEY_SEQUENCE 16

typedef enum {
  NOARROWS_THREE = 3, NOARROWS_SIX = 6
} NoArrows;
NoArrows noArrows;

typedef enum {
  SINGLE = 0, ABCD_CDAB = 1
} ABCDSequence;
ABCDSequence abcdSequence;

typedef enum {
  EMPTY, RECEIVING, UNHANDLED
} KeySequenceStatus;

struct KeySequence {
  KeySequenceStatus status = EMPTY;
  char sequence[MAX_KEY_SEQUENCE + 1];
  int noKeys = 0;
};
KeySequence keySequence;

/// \param[in] speed The desired bit rate in bits per second
/// \param[in] rxPin The pin that is used to get data from the receiver
/// \param[in] txPin The pin that is used to send data to the transmitter
/// \param[in] pttPin The pin that is connected to the transmitter controller. It will be set HIGH to enable the transmitter (unless pttInverted is true).
/// \param[in] pttInverted true if you desire the pttin to be inverted so that LOW wil enable the transmitter.
//  RH_ASK(uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false);
RH_ASK driver(2000, 11, OUTPUT_PIN, 13, false);

byte row_pins[4] = { 2, 3, 4, 5 };
byte col_pins[4] = { 6, 7, 8, 9 };
char keys[16] = { 'D', 'C', 'B', 'A', //
                  '#', '9', '6', '3', //
                  '0', '8', '5', '2', //
                  '*', '7', '4', '1'
                };
char key = 0;

int timeNextRound;
unsigned long lastTimePushed = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  if (!driver.init()) {
    Serial.println("init failed");
  }

  pinMode(LED_PIN, OUTPUT);

  noArrows = NOARROWS_THREE;
  timeNextRound = 120;

  Serial.print("Number of arrows: ");
  Serial.println(noArrows);

  abcdSequence = ABCD_CDAB;

  for (byte i = 0; i < 4; i++) {
    pinMode(row_pins[i], INPUT);
    pinMode(col_pins[i], INPUT_PULLUP);
  }
}

void loop() {
  char newKey = getKey();
  bool isPushed = newKey > 0 && key == 0;

  if (isPushed) {
    lastTimePushed = millis();
  }

  digitalWrite(LED_PIN, millis() - lastTimePushed > 300 ? HIGH : LOW);

  key = newKey;
  if (isPushed) {
    Serial.print("Key ");
    Serial.println(key);

    if (key == '*') {
      keySequence.status = RECEIVING;
      keySequence.noKeys = 0;
    } else if (key == '#') {
      keySequence.status = UNHANDLED;
      keySequence.sequence[keySequence.noKeys] = '\0';
    }

    if ((keySequence.status == RECEIVING)
        && keySequence.noKeys < MAX_KEY_SEQUENCE) {
      keySequence.sequence[keySequence.noKeys] = key;
      ++keySequence.noKeys;
    }
  }

  // Single key command
  if (isPushed && keySequence.status == EMPTY) {
    if (key == 'A') {
      sendStart(); // Start
    } else if (key == 'B') {
      sendMessage("P"); // Pause
    } else if (key == 'C') {
      sendMessage("C"); // Continue
    } else if (key == 'D') {
      sendMessage("I"); // Interrupt
    } else if (key == '0') {
      sendMessage("Z1"); // Sound
    } else if (key == '1') {
      sendMessage("F"); // Increase time with 40 seconds
    } else if (key == '2') {
      sendMessage("I"); // Decrease time with 40 seconds
    } else if (key == '7') {
      sendMessage("A1"); // AB next
    } else if (key == '8') {
      sendMessage("A2"); // CD next
    }
  }

  // Handle the key sequence
  if (keySequence.status == UNHANDLED) {
    handleKeySequence();
    emptyKeySequence();
  }
}

// TODO obsolete
void sendTime(int timeSeconds) {
  char msg[6];
  msg[0] = 'T';
  msg[1] = ' ';
  sprintf(&msg[2], "%d", timeSeconds);
  msg[5] = '\0';
  sendMessage(msg);
  delay(500);
}

void sendStart() {
  sendMessage("S 000");
  timeNextRound = noArrows == NOARROWS_THREE ? 120 : 240;
}

void sendMessage(const char* msg) {
  Serial.print("Sending message [");
  Serial.print(msg);
  Serial.println("]");

  driver.send((uint8_t *)msg, strlen(msg) + 1);
  driver.waitPacketSent();
}

void handleKeySequence() {
  Serial.print("Key sequence: ");
  Serial.println(keySequence.sequence);

  if (strcmp(keySequence.sequence, "*3") == 0) {
    Serial.println("Three arrows");
    noArrows = NOARROWS_THREE;
    timeNextRound = 120;
    sendTime(timeNextRound);
  } else if (strcmp(keySequence.sequence, "*6") == 0) {
    Serial.println("Six arrows");
    noArrows = NOARROWS_SIX;
    timeNextRound = 240;
    sendTime(timeNextRound);
  }

  if (strcmp(keySequence.sequence, "*ABC") == 0) {
    Serial.println("Single round");
    abcdSequence = SINGLE;
    sendMessage("V SINGLE");
  } else if (strcmp(keySequence.sequence, "*ABCD") == 0) {
    Serial.println("ABCD CDAB");
    abcdSequence = ABCD_CDAB;
    sendMessage("V DOUBLE");
  }

  if (strcmp(keySequence.sequence, "*0") == 0) {
    sendMessage("R"); // Reset
  }

}

void emptyKeySequence() {
  keySequence.status = EMPTY;
  keySequence.noKeys = 0;
  keySequence.sequence[0] = '\0';
}

char getKey() {
  char key = 0;
  for (byte row = 0; row < 4; row++) {
    pinMode(row_pins[row], OUTPUT);
    digitalWrite(row_pins[row], LOW);
    for (byte col = 0; col < 4; col++) {
      if (digitalRead(col_pins[col]) == LOW) {
        key = keys[row * 4 + col];
      }
    }
    pinMode(row_pins[row], INPUT);
  }

  return key;
}

