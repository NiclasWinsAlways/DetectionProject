#include <Arduino.h>
#include "esp_sleep.h"

// GPIO pin for FC-51 sensor
#define SENSOR_PIN 33

// Variables saved during deep sleep
RTC_DATA_ATTR unsigned long lastMotionTime = 0; 
RTC_DATA_ATTR int motionCount = 0; 
RTC_DATA_ATTR unsigned long startTime = 0; 

// Additional tracking variables
static int totalMotions = 0; // Total motion events across intervals
static int intervals = 0; // Total intervals for average frequency calculation

// Timers
const unsigned long testSleepThreshold = 60000;  // 1 min for testing
// const unsigned long sleepThreshold = 300000; // 5 min for production
const unsigned long debounceDelay = 200; // Debounce time (ms)

unsigned long lastDebounceTime = 0; // Last debounce update
int previousState = HIGH; // Previous motion state
int currentDebouncedState = HIGH; // Debounced motion state

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for serial monitor

  pinMode(SENSOR_PIN, INPUT); // Set sensor as input

  // Check why the ESP woke up
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up due to motion!");
    lastMotionTime = millis(); // Reset inactivity timer
  } else {
    Serial.println("Initialized. Waiting for motion...");
    lastMotionTime = millis(); // Reset timer
    startTime = millis(); // Start frequency timer
    motionCount = 0; // Reset motion count
  }

  // Set wake-up source
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW); // Wake on motion
}

void loop() {
  int rawState = digitalRead(SENSOR_PIN); // Read sensor state

  // Debounce logic
  if (rawState != currentDebouncedState) {
    if (millis() - lastDebounceTime > debounceDelay) {
      currentDebouncedState = rawState; 
      lastDebounceTime = millis(); 

      // If motion detected, log it
      if (currentDebouncedState == LOW) {
        unsigned long eventTime = millis(); // Get the timestamp
        Serial.print("Motion detected at: ");
        Serial.print(eventTime / 1000); // Time in seconds
        Serial.println(" seconds.");

        lastMotionTime = millis(); 
        motionCount++; 
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
    float avgFrequency = (float)totalMotions / intervals; // Average frequency
    Serial.print("Average motion frequency: ");
    Serial.print(avgFrequency, 2);
    Serial.println(" events per minute.");

    motionCount = 0; 
    startTime = currentTime; 
  }

  // Check inactivity and sleep if no motion
  if (millis() - lastMotionTime >= testSleepThreshold) {
    Serial.println("No motion for 1 minute. Going to sleep...");
    delay(100); 
    esp_deep_sleep_start();
  }

  delay(50); // Stability delay
}
