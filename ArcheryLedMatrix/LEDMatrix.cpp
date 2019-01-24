// -*- mode: C++ -*-

/*
  Members of class LEDMatrix
*/

#include "LEDMatrix.h"
#include <FastLED.h>
#include "asciifont.h"

LEDMatrix::LEDMatrix(const int sX, const int sY, CRGB* ledArray) {
  sizeX = sX;
  sizeY = sY;
  leds = ledArray;
}

void LEDMatrix::setAllLedsHSV(const uint8_t hue, const uint8_t saturation,
                              const uint8_t intensity) {
  for (int i = 0; i < sizeX * sizeY; i++) {
    leds[i] = CHSV(hue, saturation, intensity);
  }
  FastLED.show();
}

void LEDMatrix::setAllLedsRGB(const uint8_t red, const uint8_t green,
                              const uint8_t blue) {
  for (int i = 0; i < sizeX * sizeY; i++) {
    leds[i] = CRGB(red, green, blue);
  }
  FastLED.show();
}


int LEDMatrix::pixel(int x, int y) {
  int lednumber;
  if (x % 2 == 0) {
    lednumber = 8 * x + y;
  }
  else {
    lednumber = 8 * x + (7 - y);
  }
  return lednumber;
}

void LEDMatrix::showCharacterHSV(unsigned char c, float x_pos, int hue, int sat, int val) {
  int ind0 = (c - 28) * 5;

  for (int x = 0; x < 5; x++) {
    unsigned char fontCol = Font5x7[ind0 + x];
    unsigned char p = 1;

    for (int y = 0; y < sizeY; y++) {

      int isOn = ((int)(fontCol & p) != 0);
      int px = pixel(x_pos + x, y);
      if (px >= 0 && px < (sizeX * sizeY)) {
        leds[px].setHSV(hue, sat, isOn * val);
      }
      p = p << 1;
    }
  }
}

void LEDMatrix::showTextHSV(char* str, float x_pos, int hue, int sat, int val) {
  strncpy(text, str, maxCharInText);
  text[maxCharInText] = '\0';
  timeWhenCalled = millis();
  startX_pos = x_pos;
  this->hue = hue;
  this->sat = sat;
  this->val = val;
}

void LEDMatrix::updateText() {
  int len = strlen(text);
  if (!scroll) {
    for (int i = 0; i < len; i++) {
      showCharacterHSV(text[i], startX_pos + i * 6, hue, sat, val);
    }
  }
  else {
    FastLED.clear();
    unsigned long t = millis();
    for (int i = 0; i < len; i++) {
      showCharacterHSV(text[i], startX_pos - (t - timeWhenCalled) * 0.01 + i * 6, hue, sat, val);
    }
  }
}

void LEDMatrix::setScroll(bool scrollOn) {
  scroll = scrollOn;
}

void LEDMatrix::drawRectangleRGB(int x0, int y0, int width, int height, int R, int G, int B) {
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      leds[pixel(x0 + i, y0 + j)] = CRGB(R, G, B);
    }
  }
}

void LEDMatrix::drawRectangleHSV(int x0, int y0, int width, int height, int hue, int sat, int val) {
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      leds[pixel(x0 + i, y0 + j)] = CHSV(hue, sat, val);
    }
  }
}

