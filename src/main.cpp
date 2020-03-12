/*
 * Hermes LED shoes
 * Copyright 2013-2014 RGAM LLC
 *
 */

#include <Arduino.h>
/* Run parameters: */
#define MAX_BRIGHTNESS 32 // Max LED brightness.
#define MIN_BRIGHTNESS 16
#define WAIT_FOR_KEYBOARD 0 // Use keyboard to pause/resume program.

/* Neopixel parameters: */
#define LED_COUNT 19
// #define DATA_PIN 3
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

/* Animation parameters: */
// ~15 ms minimum crawl speed for normal mode,
// ~2 ms minimum for superfast hack mode.
#define CRAWL_SPEED_MS 8
// General sensitivity of the animation.
// Raising this raises the vector magnitude needed to reach max (purple),
// and thus lowers sensitivity.
// Eg: 800 = more sensitive, 1600 = less sensitive
#define HERMES_SENSITIVITY 1.0
// Emulate two strips by starting the crawl in the
// middle of the strip and crawling both ways.
#define ENABLE_SPLIT_STRIP 1
// Center LED, aka LED #0.
#define SPLIT_STRIP_CENTER 8

/* Sleeping parameters: */
#define SLEEP_BRIGHTNESS 0.30
#define SLEEP_CYCLE_MS 5000 // 5 second breathing cycle.
#define SLEEP_WAIT_TIME_MS 5000 // No movement for 5 seconds triggers breathing.
#define SLEEP_SENSITIVITY 25

/* Debug parameters: */
#define PRINT_LOOP_TIME 0

/* Advanced: */
#define ONBOARD_LED_PIN 2 // Pin D7 has an LED connected on FLORA.

///////////////////////////////////////////////////////////////////

// LED imports.'s
// #include <Adafruit_NeoPixel.h>
#include <FastLED.h>

// Accel imports.
#include <Wire.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_LSM303_Old.h>
// #include <Adafruit_LSM303.h>
#include <Adafruit_MPU6050.h>

// #include <LSM303.h>

// Our custom data type.
#include "AccelReading.h"

Adafruit_MPU6050 accel;
// accel.setFilterBandwidth(MPU6050_BAND_21_HZ);

void colorSetup();        
void accelSetup();        
void loopDebug();     
void accelPoll();      
void updateLED();      
void pauseOnKeystroke();                  
void showColorOff();
void showColor(float scale);         
int bufferSize();                          
void calibrate();      
void showCalibration();            
bool fillBuffer();
void printBuffer();
void printDelta();
void printMagnitude();
double getMagnitude(AccelReading reading);
void resetBreathe();
void colorOff();
void stripShow();
AccelReading getPreviousReading();
AccelReading getCurrentReading();
bool equalReadings(AccelReading a, AccelReading b);
uint32_t pixelColorForScale(double scale);
bool sleep();
void breathe();
int constrainBetween(int value, int lower, int higher);
uint32_t color(int color, float brightness);
void crawlColor(uint32_t color);
                    

void setup() {
  Serial.begin(9600);
  Serial.println("Starting hermes");
  if (WAIT_FOR_KEYBOARD) {
    // Wait for serial to initalize.
    while (!Serial) { }

  	Serial.println("Strike any key to start...");

  	// Wait for the next keystroke.
  	while (!Serial.available()) { }

  	// Clear the serial buffer.
    Serial.read();
  }
  
  colorSetup();
  
  accelSetup();
 
}

void loop() {
  loopDebug();

  accelPoll();
  updateLED();
}

// Debug functions controlled by run/debug parameters.
unsigned long before = 0;
void loopDebug() {
  if (WAIT_FOR_KEYBOARD) {
    pauseOnKeystroke();
  }
  if (PRINT_LOOP_TIME) {
    unsigned long now = millis();
    Serial.println(now - before);
    before = millis();
  }
}

void pauseOnKeystroke() {
  if (Serial.available()) {
    // Clear the serial buffer.
    Serial.read();

    Serial.println("Paused. Strike any key to resume...");

    // Turn all LEDs off.
    showColorOff();

    // Wait for the next keystroke.
    while (!Serial.available()) { }

    // Clear the serial buffer.
    Serial.read();
  }
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// accel //
///////////

// Adafruit_LSM303_Old lsm; // Bridge to accelerometer hardware.
//Adafruit_LSM303 lsm; // Bridge to accelerometer hardware.
AccelReading accelBuffer[10]; // Buffer for storing the last 10 readings.
int bufferPosition; // Current read position of the buffer.

double calibration; // Baseline for accelerometer data.
unsigned long calibrationLEDTime;
bool calibrationLEDOn;

// For breathing, track the time of the last significant movement.
unsigned long lastSignificantMovementTime;

// Initialization.
void accelSetup() {
  if (WAIT_FOR_KEYBOARD) {
    Serial.println("BEGIN");
  }
  accel.setAccelerometerRange(MPU6050_RANGE_2_G);
  accel.setFilterBandwidth(MPU6050_BAND_21_HZ);
  accel.begin();
  
  bufferPosition = 0;

  // Initialize the full buffer to zero.
  for (int i = 0; i < bufferSize(); i++) {
    accelBuffer[i].x = 0;
    accelBuffer[i].y = 0;
    accelBuffer[i].z = 0;
  }
  
  calibrate();
}

void calibrate() {
  if (WAIT_FOR_KEYBOARD) {
    Serial.println("Calibrating");
  }

  calibration = 0;
  calibrationLEDTime = 0;
  calibrationLEDOn = false;
  
  showCalibration();

  while (1) {
    if (WAIT_FOR_KEYBOARD) {
      Serial.print("...");
    }
    // Update onboard LED.
    unsigned long now = millis();
    if (now - calibrationLEDTime > 250) {
      calibrationLEDTime = now;
      calibrationLEDOn = !calibrationLEDOn;
      digitalWrite(ONBOARD_LED_PIN, calibrationLEDOn ? HIGH : LOW);
    }
    
    // Fill the buffer.
    if(!fillBuffer()) {
      delay(10);
      if (WAIT_FOR_KEYBOARD) {
        Serial.println("Waiting to fill buffer");
      }
      continue;
    }
    
    // Check to see if we're done.
    bool pass = true;
    double avg = 0;
    for (int i = 0; i < bufferSize(); i++) {
      double m = getMagnitude(accelBuffer[i]);
      pass = pass && (abs(m - calibration) < 10);
      avg += m;
    }
    
    if (pass) {
      if (WAIT_FOR_KEYBOARD) {
        Serial.print("Calibration: ");
        Serial.println(calibration);
      }
      break;
    } else {
      avg /= bufferSize();
      calibration = avg;
      if (WAIT_FOR_KEYBOARD) {
        Serial.print("Recalculating with average: ");
        Serial.println(calibration);
      }
    }
  }
  
  // Turn the calibration light off.
  digitalWrite(ONBOARD_LED_PIN, LOW);
}

// Gathers data from accelerometer into the buffer. Only writes to the buffer
// if the hardware has gathered data since we last wrote to the buffer.
void accelPoll() {
  // Read new accelerometer data. If there is no new data, return immediately.
  if (!fillBuffer()) {
    return;
  }
  
  /* PRINT DATA: */
  // printBuffer();
  // printDelta();
  // printMagnitude();
  Serial.println();
}

// Gets the vector for the given reading.
double getVector(AccelReading reading) {
  double normalizedVector = abs(calibration - getMagnitude(reading));
  return normalizedVector;
}

///////////////////////////////////////////////////////////////////

// This may or may not fill the next buffer position. If the accelerometer hasn't
// processed a new reading since the last buffer, this function immediately exits,
// returning false.
// Otherwise, if the accelerometer has read new data, this function advances the
// buffer position, fills the buffer with accelerometer data, and returns true.
bool fillBuffer() {
  // Read from the hardware.
  sensors_event_t a, g, temp;
  accel.getEvent(&a, &g, &temp);
  
  AccelReading newReading;
  newReading.x = a.acceleration.x;
  newReading.y = a.acceleration.y;
  newReading.z = a.acceleration.z;
  
  // The accelerometer hasn't processed a new reading since the last buffer.
  // Do nothing and return false.
  if (equalReadings(getCurrentReading(), newReading)) {
    return false;
  }
  
  // The accelerometer has read new data.
  
  // Advance the buffer.
  if (++bufferPosition >= bufferSize()) {
    bufferPosition = 0;
  }

  AccelReading *mutableCurrentReading = &accelBuffer[bufferPosition];
  
  mutableCurrentReading->x = newReading.x;
  mutableCurrentReading->y = newReading.y;
  mutableCurrentReading->z = newReading.z;
  
  return true;
}

///////////////////////////////////////////////////////////////////

// Gets the average difference between the latest buffer and previous buffer.
int getDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();
  
  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));
  
  return (deltaX + deltaY + deltaZ) / 3;
}

void printDelta() {
  AccelReading previousReading = getPreviousReading();
  AccelReading currentReading  = getCurrentReading();
  
  int deltaX = abs(abs(currentReading.x) - abs(previousReading.x));
  int deltaY = abs(abs(currentReading.y) - abs(previousReading.y));
  int deltaZ = abs(abs(currentReading.z) - abs(previousReading.z));

  Serial.print("deltaX: "); Serial.print(deltaX); Serial.print("\t");
  Serial.print("deltaY: "); Serial.print(deltaY); Serial.print("\t");
  Serial.print("deltaZ: "); Serial.print(deltaZ); Serial.print("\t");
  Serial.print(getDelta()); Serial.println();
}

// Gets the vector magnitude for the given reading.
// http://en.wikipedia.org/wiki/Euclidean_vector#Length
double getMagnitude(AccelReading reading) {
  double x = reading.x;
  double y = reading.y;
  double z = reading.z;

  double vector = x * x + y * y + z * z;

  return sqrt(vector);
}

void printMagnitude() {
  Serial.print("Magnitude: "); Serial.println(getMagnitude(getCurrentReading()));
}

// Prints the latest buffer reading to the screen.
void printBuffer() {
  Serial.print("accelBuffer.x: "); Serial.print(accelBuffer[bufferPosition].x); Serial.print("\t");
  Serial.print("accelBuffer.y: "); Serial.print(accelBuffer[bufferPosition].y); Serial.print("\t");
  Serial.print("accelBuffer.z: "); Serial.print(accelBuffer[bufferPosition].z); Serial.println();
}

///////////////////////////////////////////////////////////////////

// Returns the number of items held by the buffer.
int bufferSize() {
  return sizeof(accelBuffer) / sizeof(accelBuffer[0]);
}

AccelReading getCurrentReading() {
  return accelBuffer[bufferPosition];
}

// Gets the previous buffer reading.
AccelReading getPreviousReading() {
  int previous = bufferPosition - 1;
  if (previous < 0) previous = bufferSize() - 1;
  return accelBuffer[previous];
}

// Returns true if two readings are equal.
bool equalReadings(AccelReading a, AccelReading b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}


///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// color //
///////////

int COLOR_RANGE = 255;
uint32_t lastColor;
unsigned long lastCrawl;
uint32_t lightArray[LED_COUNT];

// Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, DATA_PIN, NEO_GRB + NEO_KHZ800);
CRGB strip[LED_COUNT];

void colorSetup() {
  FastLED.addLeds<LED_TYPE, 3, COLOR_ORDER>(strip, LED_COUNT);
  FastLED.setBrightness(MIN_BRIGHTNESS);
  lastColor = 0;
  lastCrawl = 0;
  
  // Turn the strip on.
  stripShow();
  
  // Initialize the LED buffer.
  for (int i = 0; i < LED_COUNT; i++) {
    lightArray[i] = 0;
  }
}

void updateLED() {
  // LED color takes a value from 0.0 to 1.0. Calculate scale from the current vector.

  // Largest vector needed to hit max color (1.0).
  double upperBound = HERMES_SENSITIVITY;
  double normalizedVector = abs(calibration - getMagnitude(getCurrentReading()));
  double scale = normalizedVector / upperBound;
  
  uint32_t pixelColor = pixelColorForScale(scale);
  
  // Change LED strip color.
  showColor(scale);
  
  if (sleep()) {
    breathe();
  } else {
    crawlColor(pixelColor);
  }
}

// "Crawls" the given color along the strip.
// This always sets LED[0] to the given color.
// After CRAWL_SPEED_MS milliseconds,
// we set LED[n + 1] = LED[n] for each LED.
void crawlColor(uint32_t color) {
  
  // Set the head pixel to the new color.
  uint32_t head = lightArray[0];
  lightArray[0] = color;
  
  unsigned long now = millis();
  
  // Shift the array if it's been long enough since last shifting,
  // or if a new color arrives.
  bool shouldUpdate = 
      (now - lastCrawl > CRAWL_SPEED_MS)
      || (color != head);

  if (!shouldUpdate) {
    return;
  }

  lastCrawl = now;
  
  // Shift the array.
  for (int i = LED_COUNT - 1; i > 0; --i) {
    lightArray[i] = lightArray[i - 1];
  }

  if (ENABLE_SPLIT_STRIP) {
    int centerLED = SPLIT_STRIP_CENTER;
    int LEDsPerSide = floor(LED_COUNT / 2);
  
    // Crawl 'low' side (center down)
    uint32_t *pixelColor = lightArray;
    for (int led = centerLED - 1; led >= centerLED - 1 - LEDsPerSide; led--) {
      strip[constrainBetween(led, 0, LED_COUNT - 1)].setHue(*pixelColor++);
    }
  
    // Crawl 'high' side (center up)
    pixelColor = lightArray;
    for (int led = centerLED; led < centerLED + LEDsPerSide; led++) {
      strip[constrainBetween(led, 0, LED_COUNT - 1)].setHue(*pixelColor++);
    }
  
    stripShow();
    
    return;
  }
  
  for (int i = 0; i < LED_COUNT; i++) {
    strip[i] = lightArray[i];
  }
  stripShow();
}

int constrainBetween(int value, int lower, int higher) {
  if (value < lower) {
    value = higher - (lower - value) + 1;
  } else if (value > higher) {
    value = lower + (value - higher) - 1;
  }
  return value;
}

// Sets the strip all one color.
// Scale parameter is a value 0.0 to 1.0,
// representing how far on the rainbow to go.
void showColor(float scale) {
  uint32_t pixelColor = pixelColorForScale(scale);

  if (pixelColor == lastColor) {
    // No change since we last set the pixels; don't bother changing them.
    return;
  }
  lastColor = pixelColor;

  // Serial.print("Show "); Serial.print(scale); Serial.println(c);
  for (int i = 0; i < LED_COUNT; i++) {
   strip[i].setHue(pixelColor);
  }
  stripShow();
}

// Returns a pixel color for use by strip[)] =
// Automatically adjusts brightness.
// Takes a scale, from 0.0 to 1.0, indicating progression
// through the color rainbow.
uint32_t pixelColorForScale(double scale) {
  int c = COLOR_RANGE * scale; // Intentionally round to an int.

  // return color(c, brightness);
  Serial.print("c: "); Serial.print(c); Serial.print("  "); Serial.print("scale: "); Serial.println(scale); 
  return c;
}

// Color 1 from 384; brightness 0.0 to 1.0.
uint32_t color(int color, float brightness)  {
  // Our logic is 0 - 383
  // color = min(max(color, 0), 255);
  return color;
}

void showColorOff() {
  colorOff();
  stripShow();
}

void colorOff() {
  for (int i = 0; i < LED_COUNT; i++) {
    strip[i] = 0;
  }
}

// Show the calibration colors.
void showCalibration() {
  colorOff();

  int mid = LED_COUNT / 2;
  
  // Red
  strip[mid - 1] = CRGB::Red;
  // Green
  strip[mid] = CRGB::Green;
  // Blue
  strip[mid + 1] = CRGB::Blue;
  
  stripShow();
}

void stripShow() {
  FastLED.show();
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

///////////
// sleep //
///////////
bool sleeping = false;

bool sleep() {
  unsigned long now = millis();

  // See if this movement is significant, aka enough to wake us from sleep.
  double m = getMagnitude(getCurrentReading());
  if (abs(calibration - m) > SLEEP_SENSITIVITY) {
    lastSignificantMovementTime = now;
  }
  
  // Last significant movement time needs to be longer than sleep wait time.
  if (now - lastSignificantMovementTime < SLEEP_WAIT_TIME_MS) {
    // Haven't waited long enough.
    resetBreathe();
    sleeping = false;
    return false;
  }
  
  // Only start sleeping on the sleep period.
  if (!sleeping && (now % SLEEP_CYCLE_MS != 0)) {
    resetBreathe();
    sleeping = false;
    return false;
  }
  
  sleeping = true;

  return true;
}

///////////////////////////////////////////////////////////////////

const uint8_t KEYFRAMES[]  = {
  // Rising
  20, 21, 22, 24, 26, 28, 31, 34, 38, 41, 45, 50, 55, 60, 66, 73, 80, 87, 95,
  103, 112, 121, 131, 141, 151, 161, 172, 182, 192, 202, 211, 220, 228, 236,
  242, 247, 251, 254, 255,

  // Falling
  254, 251, 247, 242, 236, 228, 220, 211, 202, 192, 182, 172, 161, 151, 141,
  131, 121, 112, 103, 95, 87, 80, 73, 66, 60, 55, 50, 45, 41, 38, 34, 31, 28,
  26, 24, 22, 21, 20,
  20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 
};

unsigned long lastBreath = 0.0;
int keyframePointer = 0;

void resetBreathe() {
  keyframePointer = 0;
}

void breathe() {
  int numKeyframes = sizeof(KEYFRAMES) - 1;
  float period = SLEEP_CYCLE_MS / numKeyframes;
  unsigned long now = millis();
  
  if ((now - lastBreath) > period) {
    lastBreath = now;

    for (int i = 0; i < LED_COUNT; i++) {
      uint8_t color = (SLEEP_BRIGHTNESS * 127 * KEYFRAMES[keyframePointer]) / 256;
      strip[i].setHue(color);
    }
    FastLED.show();   

    // Increment the keyframe pointer.
    if (++keyframePointer > numKeyframes) {
      // Reset to 0 after the last keyframe.
      keyframePointer = 0;
    }   
  }
}
