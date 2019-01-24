// -*- mode: C++ -*-
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <math.h>
//#include <EEPROM.h>
#include "LedMatrix.h"

#define LED_CONTROL_PIN 2
#define RF_RX_PIN 26
#define POWER_IND_PIN 28

#define SOUND_PIN 53

#define MAX_MESSAGE_LENGTH  32
#define MAX_COMMAND_LENGTH  8
#define MAX_SEQUENCE_SIZE   128
#define MAX_RECORDS         32
#define RECORD_LENGTH       7

#define ON          LOW
#define OFF         HIGH

#define SIZE_X  32
#define SIZE_Y  8
#define NUM_LEDS SIZE_X*SIZE_Y

#define SOUND_ENABLED true

#define PERIODIC_PULSE_INTERVAL 1000

RH_ASK driver(2000, RF_RX_PIN, 12, 10, false);

enum SequenceState {
  NONE, RUNNING, PAUSED, FINISHED
};

enum CountdownState {
  NO_COUNTDOWN, WAIT_COUNTDOWN, SHOOT_COUNTDOWN
};

enum ABCDState {
  NO_ABCD, AB, CD
};

enum sequenceType {
  SINGLE_ROUND, DOUBLE_ROUND
};

enum RoundState {
  FIRST, SECOND
};

struct Sequence {
  enum ABCDState abcdState = NO_ABCD;
  enum sequenceType sequenceType;
  enum RoundState roundState;
  enum SequenceState state = NONE;
  enum CountdownState countdownState = NO_COUNTDOWN;
  enum CountdownState lastCountdownState = NO_COUNTDOWN;
  int greenDuration = 90;
  int lastGreenDuration = 0;
  int recordCounter = 0;
  int lastHandledRecordCounter = -1;
  unsigned long startTime;
  unsigned long timeRunningSequence;
  unsigned long timeRunningReal;
  char currentSequence[MAX_SEQUENCE_SIZE];
  unsigned long scheduledRecordStartTimes[MAX_RECORDS];
  long timeShootingCountdown;
  int timeShootingCountdownSeconds; //
  bool timeShootingCountdownSecondsChangedPulse = false; //
  long timeWaitingCountdown;
  int timeWaitingCountdownSeconds; //
  bool timeWaitingCountdownSecondsChangedPulse = false; //
  bool isShooting = false; //
  bool isShootingChangedPulse = false; //
  bool sequenceIsRunningChangedPulse = false;
  int arrowsToShoot;
  int noArrowsLeft; //
  bool noArrowsChangedPulse = false; //
  bool changedABCDPulse = false; //
  bool shootingPaused = false; //
};

struct Sound {
  int noSignals;
  unsigned int duration = 800;
  boolean sounding = false;
  unsigned long startTime = 0;
};
struct Sequence _sequence;
struct Sound _sound;

// Received message definitions which changes state or sequence
// "T ttt": Set shooting time to ttt
// "S ttt": Start sequence with shooting time ttt. If ttt is 0, previous time set is used
// "F: Increase green time with 40 seconds
// "D": Decrease green time with 40 seconds
// "A1": AB light on
// "A2": CD light on
// "ZN": N sound signals
// "I": Interrupt
// "R": Reset to default (red light)
// "P": Pause
// "C": Continue
// "V SINGLE": Single round mode
// "V DOUBLE": Double round mode (AB-CD/CD-AB)

// Messages for simple operation types without status or sequence effects
// 'A': Single sound signals
// 'B': Two sound signals
// 'C': Three sound signals
// 'G': Green light on, others off
// 'Y': Yellow light on, others off
// 'R': Red light on, others off
// 'Z': End
// 'E': Empty placeholder

char _singleRoundTemplate[9 * 7] = { //
  'E', ' ', ' ', '0', '\0', ' ', ' ', // Start
  'B', ' ', ' ', '0', '\0', ' ', ' ', // Two sound signals
  'W', ' ', '1', '0', '\0', ' ', ' ', // Red light for 10 seconds with countdown
  'A', ' ', ' ', '0', '\0', ' ', ' ', // One sound signal
  'G', 'T', 'T', 'T', '\0', 'I', ' ', // Green light for TTT seconds, interruptable
  'Y', ' ', '3', '0', '\0', 'I', 'S', // Yellow light for 30 seconds, interruptable, skipable
  'R', ' ', ' ', '0', '\0', ' ', ' ', // Red light
  'C', ' ', ' ', '0', '\0', ' ', ' ', // Three sound signals
  'Z', ' ', ' ', '0', '\0', ' ', ' '  // End
};

char _doubleRoundTemplate[15 * 7] = { //
  'E', ' ', ' ', '0', '\0', ' ', ' ', // Start
  'B', ' ', ' ', '0', '\0', ' ', ' ', // Two sound signals
  'W', ' ', '1', '0', '\0', ' ', ' ', // Red light for 10 seconds with countdown
  'A', ' ', ' ', '0', '\0', ' ', ' ', // One sound signal
  'G', 'T', 'T', 'T', '\0', 'I', ' ', // Green light for TTT seconds, interruptable
  'Y', ' ', '3', '0', '\0', 'I', 'S', // Yellow light for 30 seconds, interruptable, skipable
  'B', ' ', ' ', '0', '\0', ' ', ' ', // Two sound signals
  'S', ' ', ' ', '0', '\0', ' ', ' ', // Switch AB-CD
  'W', ' ', '1', '0', '\0', ' ', ' ', // Red light for 10 seconds with countdown
  'A', ' ', ' ', '0', '\0', ' ', ' ', // One sound signal
  'G', 'T', 'T', 'T', '\0', 'I', ' ', // Green light for TTT seconds, interruptable
  'Y', ' ', '3', '0', '\0', 'I', 'S', // Yellow light for 30 seconds, interruptable, skipable
  'R', ' ', ' ', '0', '\0', ' ', ' ', // Red light
  'C', ' ', ' ', '0', '\0', ' ', ' ', // Three sound signals
  'Z', ' ', ' ', '0', '\0', ' ', ' ' // End
};

bool _lightABIsOn = true;

unsigned long _dt;
unsigned long _tPrev;
unsigned long _t;
unsigned long _timeOfLastStatus;
unsigned long _timeOfLastMessage = 0;

int _ledIntensity = 127;
int _waitNumberHue = 200;
int _shootNumberHue = 200;
int _abcdHue = 170;
int _numbersXPos = 10;

uint8_t _messageBuffer[MAX_MESSAGE_LENGTH];

CRGB _leds[NUM_LEDS];
LEDMatrix* _ledMatrixP;

int _hueGreen = 100;
int _hueYellow = 50;
int _hueRed = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Start ArcheryLedMatrix");

  pinMode(SOUND_PIN, OUTPUT);
  pinMode(POWER_IND_PIN, OUTPUT);
  digitalWrite(SOUND_PIN, OFF);
  digitalWrite(POWER_IND_PIN, OFF);

  delay(500);

  FastLED.addLeds<NEOPIXEL, LED_CONTROL_PIN>(_leds, NUM_LEDS, 0);
  FastLED.clear();
  FastLED.show();

  _ledMatrixP = new LEDMatrix(SIZE_X, SIZE_Y, _leds);
  _ledMatrixP->setScroll(false);

  _messageBuffer[0] = '\0';

  if (!driver.init())
    Serial.println("init failed");

  _sequence.changedABCDPulse = true;

  _t = millis();
  _timeOfLastStatus = _t;

  redLightOn();
  showShootDuration(&_sequence);

  setsequenceType("V SINGLE", &_sequence);
  showABCDStatus();
}

void loop() {
  _tPrev = _t;
  _t = millis();
  _dt = _t - _tPrev;

  updateTimes(&_sequence);
  updateSequence(&_sequence);

  bool gotNewMessage = receiveRFData(_messageBuffer);

  if (gotNewMessage) {
    _timeOfLastMessage = _t;
    Serial.println((char*) _messageBuffer);
    handleMessage(_messageBuffer, &_sequence);
  }

  updateLEDPanel(&_sequence);

  digitalWrite(POWER_IND_PIN, _t - _timeOfLastMessage < 500 ? LOW : HIGH);
  _sequence.lastCountdownState = _sequence.countdownState;
}

bool receiveRFData(uint8_t* buf) {
  uint8_t buflen = MAX_MESSAGE_LENGTH;

  if (driver.recv(buf, &buflen)) {
    buf[buflen] = '\0';
    Serial.print("Got message: ");
    Serial.println((char*)buf);
    return true;
  }
  return false;
}

void handleMessage(uint8_t* messageBuffer, struct Sequence * sequenceP) {
  char* msg = (char*)messageBuffer;
  char messageType = msg[0];
  switch (messageType) {
    case 'T':
      setGreenTime(msg, sequenceP);
      break;
    case 'F':
      increaseGreenTime(sequenceP);
      break;
    case 'D':
      decreaseGreenTime(sequenceP);
      break;
    case 'S':
      startSequence(msg, sequenceP);
      break;
    case 'A':
      setABCD(msg);
      break;
    case 'V':
      setsequenceType(msg, sequenceP);
      break;
    case 'Z':
      startSoundSignal(msg);
      break;
    case 'I':
      interruptSequenceRecord(sequenceP);
      break;
    case 'R':
      resetToDefault();
      break;
    case 'P':
      doPause(sequenceP);
      break;
    case 'C':
      doContinue(sequenceP);
  }
}

void setGreenTime(char* msg, struct Sequence * sequenceP) {
  sequenceP->greenDuration = atoi(msg + 2) - 30;
  Serial.print("Green duration (s): ");
  Serial.println(sequenceP->greenDuration);
}

void increaseGreenTime(struct Sequence * sequenceP) {
  sequenceP->greenDuration = min(240 - 30, sequenceP->greenDuration + 40);
  Serial.print("Green duration (s): ");
  Serial.println(sequenceP->greenDuration);
}

void decreaseGreenTime(struct Sequence * sequenceP) {
  sequenceP->greenDuration = max(40 - 30, sequenceP->greenDuration - 40);
  Serial.print("Green duration (s): ");
  Serial.println(sequenceP->greenDuration);
}

void setsequenceType(const char* msg, struct Sequence * sequenceP) {
  int cmax = 10;
  char rType[11];

  rType[cmax] = '\0';

  if (strlen(msg) > 4) {
    strncpy(rType, msg + 2, cmax);
  } else {
    return;
  }

  if (strcmp(rType, "SINGLE") == 0) {
    sequenceP->sequenceType = SINGLE_ROUND;
  } else if (strcmp(rType, "DOUBLE") == 0) {
    sequenceP->sequenceType = DOUBLE_ROUND;
  }

  showABCDStatus();
}

void startSequence(char* msg, struct Sequence * sequenceP) {
  if (sequenceP->state == PAUSED) {
    doContinue(sequenceP);
    return;
  }

  Serial.println("startSequence");
  int shootDuration = atoi(msg + 2);
  int greenDuration;
  if (shootDuration == 0) {
    greenDuration = sequenceP->greenDuration;
  } else {
    sequenceP->greenDuration = shootDuration + 30;
    greenDuration = sequenceP->greenDuration;
  }

  Serial.print("Green duration (s): ");
  Serial.println(greenDuration);

  sequenceP->arrowsToShoot = (int) (greenDuration + 30) / 40;
  sequenceP->noArrowsChangedPulse = true;
  sequenceP->timeShootingCountdown = (greenDuration + 30) * 1000L;
  sequenceP->isShooting = false;

  Serial.print("Arrows to shoot: ");
  Serial.println(sequenceP->arrowsToShoot);

  // "S ttt": Start sequence with shooting time ttt
  char *sequenceTemplatep = sequenceP->sequenceType == SINGLE_ROUND ? _singleRoundTemplate : _doubleRoundTemplate;

  boolean atEnd = false;
  int r = 0;
  long scheduledTAtRecordStart = 0;

  while (!atEnd) {
    int k0 = r * RECORD_LENGTH;
    int k;

    sequenceP->scheduledRecordStartTimes[r] = scheduledTAtRecordStart;
    for (int j = 0; j < RECORD_LENGTH; j++) {
      k = k0 + j;
      sequenceP->currentSequence[k] = sequenceTemplatep[k];
    }
    if (sequenceP->currentSequence[k0] == 'G') {
      char s[4];
      s[3] = '\0';
      sprintf(s, "%03d", greenDuration);
      sequenceP->currentSequence[k0 + 1] = s[0];
      sequenceP->currentSequence[k0 + 2] = s[1];
      sequenceP->currentSequence[k0 + 3] = s[2];
    }

    atEnd = sequenceP->currentSequence[k0] == 'Z';
    sequenceP->recordCounter = r;
    //long recordDuration = getRecordDuration(getRecord(sequenceP));
    long t = atoi(&(sequenceP->currentSequence[k0 + 1]));
    scheduledTAtRecordStart += t * 1000;
    ++r;
  }

  sequenceP->state = RUNNING;
  sequenceP->recordCounter = 0;
  sequenceP->startTime = millis();
  sequenceP->timeRunningSequence = 0;
  sequenceP->timeRunningReal = 0;
  sequenceP->sequenceIsRunningChangedPulse = true;

  printSequence(sequenceP);
}

void setABCD(char* msg) {
  _lightABIsOn = msg[1] == '1';
  showABCDStatus();
}

void startSoundSignal(char* msg) {
  int noSignals = atoi(msg + 1);
  startSoundSignal(noSignals);
}

void interruptSequenceRecord(struct Sequence * sequenceP) {
  char *record = getRecord(sequenceP);
  if (record[5] == 'I' && record[0] != 'Z') { // Record is interruptable and not the last
    do {
      ++sequenceP->recordCounter;
      record = getRecord(sequenceP);
    } while (record[6] == 'S'); // Record is skipable

    sequenceP->timeRunningSequence =
      sequenceP->scheduledRecordStartTimes[sequenceP->recordCounter];
  }
}

void resetToDefault() {
  redLightOn();
  showShootDuration(&_sequence);
  _sequence.isShooting = false;
  _sequence.isShootingChangedPulse = true;
  _sequence.state = NONE;
  _sequence.sequenceIsRunningChangedPulse = true;
  _sequence.noArrowsLeft = 0;
  _sequence.noArrowsChangedPulse = true;
  _sequence.timeShootingCountdownSeconds = _sequence.greenDuration;
  _sequence.timeShootingCountdown = _sequence.timeShootingCountdownSeconds * 1000;
  _sequence.timeShootingCountdownSecondsChangedPulse = true;
  _sequence.timeWaitingCountdownSeconds = 10;
  _sequence.timeWaitingCountdown = _sequence.timeWaitingCountdownSeconds * 1000;
  _sequence.timeWaitingCountdownSecondsChangedPulse = true;
  _sequence.shootingPaused = false;
  _sequence.countdownState = NO_COUNTDOWN;
}

void updateLEDPanel(struct Sequence * sequenceP) {
  bool hasChangedCountdownState = sequenceP->lastCountdownState != sequenceP->countdownState;

  // Show waiting time
  if (sequenceP->countdownState == WAIT_COUNTDOWN &&
      (hasChangedCountdownState || sequenceP->timeWaitingCountdownSecondsChangedPulse)) {
    showWaitingTime(sequenceP);
  }

  // Shot countown time
  if (sequenceP->countdownState == SHOOT_COUNTDOWN &&
      (hasChangedCountdownState || sequenceP->timeShootingCountdownSecondsChangedPulse)) {
    showTimeCountdown(sequenceP);
  }

  // Changed AB/CD message
  if (sequenceP->changedABCDPulse) {
    sequenceP->changedABCDPulse = false;
    showABCDStatus();
  }

  if (sequenceP->lastGreenDuration != sequenceP->greenDuration && sequenceP->countdownState == NO_COUNTDOWN) {
    showShootDuration(sequenceP);
    Serial.print("Green duration: ");
    Serial.println(sequenceP->greenDuration);
    sequenceP->lastGreenDuration = sequenceP->greenDuration;
  }
}

void showTimeCountdown(Sequence * sequenceP) {
  int t = sequenceP->timeShootingCountdownSeconds;
  t = t < 0 ? 0 : t;
  t = t > 240 ? 240 : t;
  showRightAdjNumber(t, 3, _shootNumberHue, 255, _ledIntensity);
  FastLED.show();
}

void showWaitingTime(Sequence * sequenceP) {
  int t = sequenceP->timeWaitingCountdownSeconds;
  t = t < 0 ? 0 : t;
  t = t > 30 ? 30 : t;
  showRightAdjNumber(t, 3, _waitNumberHue, 255, _ledIntensity);
  FastLED.show();
}

void showShootDuration(Sequence* sequenceP) {
  int t = sequenceP->greenDuration + 30;
  t = t < 0 ? 0 : t;
  t = t > 240 ? 240 : t;
  showRightAdjNumber(t, 3, _waitNumberHue, 255, _ledIntensity);
  FastLED.show();
}

void showRightAdjNumber(int number, int numPlaces, int hue, int sat, int val) {
  number = min(pow(10, numPlaces) - 1, number);
  Serial.println(number);

  char text[numPlaces + 1];
  for (int i = 0; i < numPlaces; i++) {
    text[i] = ' ';
  }
  text[numPlaces] = '\0';

  int numDigits = (int)log10(number) + 1;
  int numLeadingBlanks = numPlaces - numDigits;
  numLeadingBlanks = max(0, numLeadingBlanks);
  numLeadingBlanks = min(numPlaces - 1, numLeadingBlanks);

  sprintf(&text[numLeadingBlanks], "%d", number);

  Serial.print("Show right adjusted **");
  Serial.print(text);
  Serial.println("**");

  _ledMatrixP->showTextHSV(text, _numbersXPos, hue, sat, val);
  _ledMatrixP->updateText();
  FastLED.show();
}

void updateTimes(struct Sequence * sequenceP) {
  if (sequenceP->state == RUNNING) {
    sequenceP->timeRunningSequence += _dt;
  } else if (sequenceP->state == RUNNING || sequenceP->state == PAUSED) {
    sequenceP->timeRunningReal += _dt;
  } else {
    sequenceP->timeRunningSequence = 0;
    sequenceP->timeRunningReal = 0;
  }
  if (sequenceP->isShooting) {
    sequenceP->timeShootingCountdown -= (long) _dt;
  }
  if (sequenceP->countdownState == WAIT_COUNTDOWN) {
    sequenceP->timeWaitingCountdown -= (long) _dt;
  } else {
    sequenceP->timeWaitingCountdown = 0;
  }
}

void updateSequence(struct Sequence * sequenceP) {
  char *record = getRecord(sequenceP);
  if (record[0] == 'Z') {
    sequenceP->state = FINISHED;
  }

  if (sequenceP->state == RUNNING) {
    if (sequenceP->lastHandledRecordCounter != sequenceP->recordCounter) {
      doRecord(getRecord(sequenceP));
      sequenceP->lastHandledRecordCounter = sequenceP->recordCounter;
    }

    // Step to next record if time has reached scheduled time for record start
    if (sequenceP->timeRunningSequence
        >= sequenceP->scheduledRecordStartTimes[sequenceP->recordCounter
            + 1]) {
      ++sequenceP->recordCounter;
      Serial.print("Step sequence record ");
      Serial.println(sequenceP->recordCounter);
    }

  }
  updateSound();

  // Count down update
  int tCountdownSec = sequenceP->timeShootingCountdown / 1000 + 1;
  int maxTime = sequenceP->arrowsToShoot * 40;
  tCountdownSec = tCountdownSec > maxTime ? maxTime : tCountdownSec;
  tCountdownSec = sequenceP->timeShootingCountdown <= 0 ? 0 : tCountdownSec;
  sequenceP->timeShootingCountdownSecondsChangedPulse = tCountdownSec
      != sequenceP->timeShootingCountdownSeconds;
  sequenceP->timeShootingCountdownSeconds = tCountdownSec;

  // Waiting count down update
  int tWaitCountdownSec = sequenceP->timeWaitingCountdown / 1000 + 1;
  int maxWaitTime = 10;
  tWaitCountdownSec =
    tWaitCountdownSec > maxWaitTime ? maxWaitTime : tWaitCountdownSec;
  tWaitCountdownSec =
    sequenceP->timeWaitingCountdown <= 0 ? 0 : tWaitCountdownSec;
  sequenceP->timeWaitingCountdownSecondsChangedPulse = tWaitCountdownSec
      != sequenceP->timeWaitingCountdownSeconds;
  sequenceP->timeWaitingCountdownSeconds = tWaitCountdownSec;

  // Number of arrows left update
  int noArrowsLeft = (sequenceP->timeShootingCountdownSeconds - 1) / 40 + 1;
  noArrowsLeft =
    sequenceP->timeShootingCountdownSeconds == 0 ? 0 : noArrowsLeft;
  noArrowsLeft =
    noArrowsLeft > sequenceP->arrowsToShoot ?
    sequenceP->arrowsToShoot : noArrowsLeft;
  sequenceP->noArrowsChangedPulse = sequenceP->noArrowsLeft != noArrowsLeft;
  sequenceP->noArrowsLeft = noArrowsLeft;
}

void updateSound() {
  _sound.sounding = _sound.sounding
                    && _t - _sound.startTime
                    < _sound.duration * (2 * _sound.noSignals - 1);
  boolean soundNow = _sound.sounding
                     && ((_t - _sound.startTime) / _sound.duration) % 2 == 0;
  if (SOUND_ENABLED) {
    digitalWrite(SOUND_PIN, soundNow ? ON : OFF);
  }
}

void doRecord(char* record) {
  char task = record[0];
  Serial.print("** Do sequence record at time (s): ");
  Serial.print(_sequence.timeRunningSequence / 1000L);
  Serial.print("  ");
  Serial.print(_sequence.recordCounter);
  Serial.print("  ");
  Serial.println(task);
  switch (task) {
    case 'A':
      startSoundSignal(1);
      break;
    case 'B':
      startSoundSignal(2);
      break;
    case 'C':
      startSoundSignal(3);
      break;
    case 'G':
      _sequence.isShootingChangedPulse = !_sequence.isShooting;
      _sequence.isShooting = true;
      _sequence.countdownState = SHOOT_COUNTDOWN;
      greenLightOn();
      break;
    case 'Y':
      yellowLightOn();
      break;
    case 'R':
      _sequence.isShootingChangedPulse = _sequence.isShooting;
      _sequence.isShooting = false;
      _sequence.countdownState = NO_COUNTDOWN;
      _sequence.timeShootingCountdown = (_sequence.greenDuration + 30) * 1000L;
      redLightOn();
      showShootDuration(&_sequence);
      break;
    case 'W':
      _sequence.isShootingChangedPulse = _sequence.isShooting;
      _sequence.isShooting = false;
      _sequence.timeShootingCountdown = _sequence.arrowsToShoot * 40e3;
      _sequence.timeWaitingCountdown = getRecordDuration(record) * 1000;
      _sequence.countdownState = WAIT_COUNTDOWN;
      redLightOn();
      break;
    case 'S':
      switchABCD();
      break;
    case 'E':
      // nothing
      break;
    case 'Z':
      endSequence();
  }
}

void switchABCD() {
  _lightABIsOn = !_lightABIsOn;
  if (_lightABIsOn) {
    // TODO show AB
  } else {
    // TODO show CD
  }
  _sequence.changedABCDPulse = true;
}

void greenLightOn() {
  _ledMatrixP -> drawRectangleHSV(28, 0, 4, 8, 0, 0, 0);
  _ledMatrixP -> drawRectangleHSV(28, 0, 4, 9, _hueGreen, 255, _ledIntensity);
  _ledMatrixP -> drawRectangleHSV(29, 2, 2, 4, 0, 0, 0);
  FastLED.show();
}

void yellowLightOn() {
  _ledMatrixP -> drawRectangleHSV(28, 0, 4, 8, 0, 0, 0);
  _ledMatrixP -> drawRectangleHSV(28, 0, 4, 8, _hueYellow, 255, _ledIntensity);
  FastLED.show();
}

void redLightOn() {
  _ledMatrixP -> drawRectangleHSV(28, 0, 4, 8, 0, 0, 0);
  _ledMatrixP -> drawRectangleHSV(28, 0, 4, 8, _hueRed, 255, _ledIntensity);
  FastLED.show();
}

void endSequence() {
  Serial.println("End sequence");
  resetToDefault();
}

void printSequence(struct Sequence * sequenceP) {
  Serial.println("The sequence:");
  boolean atEnd = false;
  int r = 0;
  char *c = sequenceP->currentSequence;
  while (!atEnd) {
    int k0 = r * RECORD_LENGTH;
    for (int i = 0; i < RECORD_LENGTH; i++) {
      char s = c[k0 + i];
      if (s == 0) {
        s = ' ';
      }
      Serial.print(s);
    }
    Serial.print(" - starts at ");
    long t = (sequenceP->scheduledRecordStartTimes[r]) / 1000;
    Serial.print(t);
    Serial.println("s");

    atEnd = c[k0] == 'Z';
    ++r;
  }
}

long getRecordDuration(char *record) {
  return atoi(record + 1);
}

char* getRecord(struct Sequence * sequenceP) {
  return &(sequenceP->currentSequence[sequenceP->recordCounter * RECORD_LENGTH]);
}

void startSoundSignal(int noSignals) {
  Serial.print("Start sound signal ");
  Serial.println(noSignals);
  _sound.sounding = true;
  _sound.noSignals = noSignals;
  _sound.startTime = _t;
}

void doPause(struct Sequence * sequenceP) {
  if (sequenceP->state == RUNNING) {
    sequenceP->state = PAUSED;
    sequenceP->shootingPaused = sequenceP->isShooting;
    if (sequenceP->shootingPaused) {
      sequenceP->isShooting = false;
      sequenceP->isShootingChangedPulse = true;
    }
  }
}

void doContinue(struct Sequence * sequenceP) {
  if (sequenceP->state == PAUSED) {
    sequenceP->state = RUNNING;
    sequenceP->isShooting = sequenceP->shootingPaused;
    sequenceP->isShootingChangedPulse = sequenceP->isShooting;
    sequenceP->shootingPaused = false;
  }
}

void showABCDStatus() {
  if (_sequence.sequenceType == SINGLE_ROUND) {
    _ledMatrixP -> drawRectangleHSV(0, 0, 10, 8, 0, 0, 0);
    FastLED.show();
    return;
  }

  if (_lightABIsOn) {
    char a[3];
    a[0] = 28;
    a[1] = '\0';
    char b[3];
    b[0] = 29;
    b[1] = '\0';
    _ledMatrixP->showTextHSV(a, 0, _abcdHue, 255, _ledIntensity * 2 / 3);
    _ledMatrixP->updateText();
    _ledMatrixP->showTextHSV(b, 5, _abcdHue, 255, _ledIntensity * 2 / 3);
    _ledMatrixP->updateText();
    FastLED.show();
  } else {
    char c[3];
    c[0] = 30;
    c[1] = '\0';
    char d[3];
    d[0] = 31;
    d[1] = '\0';
    _ledMatrixP->showTextHSV(c, 0, _abcdHue, 255, _ledIntensity * 2 / 3);
    _ledMatrixP->updateText();
    _ledMatrixP->showTextHSV(d, 5, _abcdHue, 255, _ledIntensity * 2 / 3);
    _ledMatrixP->updateText();
    FastLED.show();
  }


}

