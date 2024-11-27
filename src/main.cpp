#include <Arduino.h>
#include "esp_sleep.h"

// GPIO pin for FC-51 sensor
#define SENSOR_PIN 33

// Variables saved during deep sleep
RTC_DATA_ATTR unsigned long lastMotionTime = 0; 
RTC_DATA_ATTR int motionCount = 0; // Motion events
RTC_DATA_ATTR int peopleCount = 0; // Count distinct people/entities
RTC_DATA_ATTR unsigned long startTime = 0; 

// Additional tracking variables
static int totalMotions = 0; 
static int intervals = 0; 

// Timers
const unsigned long testSleepThreshold = 60000;  // 1 min for testing
const unsigned long debounceDelay = 200; // Debounce time (ms)

unsigned long lastDebounceTime = 0; 
int previousState = HIGH; 
int currentDebouncedState = HIGH; 
bool motionActive = false; // Tracks if motion is currently ongoing

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor

  pinMode(SENSOR_PIN, INPUT); 

  // Check why the ESP woke up
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up due to motion!");
    lastMotionTime = millis(); 
  } else {
    Serial.println("Initialized. Waiting for motion...");
    lastMotionTime = millis(); 
    startTime = millis(); 
    motionCount = 0; 
    peopleCount = 0; // Reset people count
  }

  // Set wake-up source
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW); 
}

void loop() {
  int rawState = digitalRead(SENSOR_PIN); 

  // Debounce logic
  if (rawState != currentDebouncedState) {
    if (millis() - lastDebounceTime > debounceDelay) {
      currentDebouncedState = rawState; 
      lastDebounceTime = millis(); 

      // Detect motion start (HIGH → LOW)
      if (currentDebouncedState == LOW) {
        motionActive = true;
        lastMotionTime = millis(); 
        motionCount++;
        Serial.println("Motion started!");
      }

      // Detect motion end (LOW → HIGH)
      if (currentDebouncedState == HIGH && motionActive) {
        motionActive = false;
        peopleCount++; // Count a distinct person passing by
        Serial.print("Person passed. Total count: ");
        Serial.println(peopleCount);
      }
    }
  }

  // Calculate motion frequency every minute
  unsigned long currentTime = millis();
  if (currentTime - startTime >= testSleepThreshold) {
    float frequency = (float)motionCount / ((currentTime - startTime) / 60000.0);
    Serial.print("Motion frequency: ");
    Serial.print(frequency, 2);
    Serial.println(" events per minute.");

    // Update total motions and intervals
    totalMotions += motionCount;
    intervals++;
    float avgFrequency = (float)totalMotions / intervals;
    Serial.print("Average motion frequency: ");
    Serial.print(avgFrequency, 2);
    Serial.println(" events per minute.");

    motionCount = 0; 
    startTime = currentTime; 
  }

  // Check inactivity and sleep if no motion
  if (millis() - lastMotionTime >= testSleepThreshold) {
    Serial.println("No motion for 1 minute. Going to sleep...");
    Serial.print("Total people counted: ");
    Serial.println(peopleCount);
    delay(100); 
    esp_deep_sleep_start();
  }

  delay(50); 
}
