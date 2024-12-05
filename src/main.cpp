#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include "esp_sleep.h"
#include <PubSubClient.h>

//use this in a cmd
//for better security make a more secure topic name and not motion/data not very secure can easily be posted to
// mosquitto_sub -h test.mosquitto.org -p 1883 -t "motion/data" -v
// Wi-Fi credentials
const char* ssid = "E308";
const char* password = "98806829";

// GPIO pin for FC-51 sensor
#define SENSOR_PIN 33

// Web server on port 80
WebServer server(80);

// ======== VARIABLES ========

// ======== DEEP SLEEP VARIABLES ========
// Variables retained during deep sleep to preserve state across wake-ups
RTC_DATA_ATTR unsigned long lastMotionTime = 0; // Time of the last motion event
RTC_DATA_ATTR int motionCount = 0;             // Number of motion events since last wake-up
RTC_DATA_ATTR int peopleCount = 0;             // Total people counted since start
RTC_DATA_ATTR unsigned long startTime = 0;     // Time when the device started or woke up

// ======== MOTION TRACKING ========
// Variables used to track motion activity and statistics
static int totalMotions = 0;                   // Total motions detected across all sessions
static int intervals = 0;                      // Count of logging intervals (e.g., every 10 seconds)

// ======== TIMERS ========
// Timing variables for periodic tasks and debounce logic
unsigned long lastLogTime = 0;                 // Tracks the last time data was logged
const unsigned long logInterval = 10000;       // Interval for periodic logging (10 seconds)
const unsigned long sleepInterval = 60000;     // Time of inactivity before going to sleep (1 minute)
const unsigned long debounceDelay = 200;       // Debounce time to filter out false motion triggers (200 ms)
unsigned long lastReconnectAttempt = 0;        // Time of the last MQTT reconnection attempt
const unsigned long reconnectInterval = 60000; // Interval for MQTT reconnection attempts (1 minute)
unsigned long lastPublishTime = 0;             // Tracks the last time data was published to MQTT
const unsigned long publishInterval = 1000;    // Interval for publishing MQTT messages (1 second)
unsigned long lastDebounceTime = 0;            // Tracks the last time the sensor input was debounced

// ======== SENSOR STATE ========
// Variables for sensor input and motion activity state
int previousState = HIGH;                      // Previous state of the motion sensor (HIGH = no motion)
int currentDebouncedState = HIGH;              // Current debounced state of the sensor
bool motionActive = false;                     // Indicates if motion is actively detected

// ======== MQTT CONNECTION STATE ========
// Tracks the connection state with the MQTT broker
bool mqttConnected = false;                    // True if connected to the MQTT broker



// MQTT broker details
const char* mqttServer = "test.mosquitto.org"; // Public test broker
const int mqttPort = 1883; // Public Test pORT
const char* mqttTopic = "motion/data"; // Public test Topic

WiFiClient espClient;
PubSubClient mqttClient(espClient);
// Add the function prototype
void connectToMQTT();

void ensureMQTTConnection() {
    if (!mqttClient.connected()) {
        Serial.print("Reconnecting to MQTT... ");
        String clientID = "ESP32Client-" + String(random(0xffff), HEX); // Unique client ID
        if (mqttClient.connect(clientID.c_str())) {
            Serial.println("Reconnected to MQTT broker!");
            mqttConnected = true;
        } else {
            Serial.print("Failed to reconnect. State: ");
            Serial.println(mqttClient.state());
            mqttConnected = false;
        }
    }
    mqttClient.loop(); // Ensure MQTT connection remains alive
}


void connectToMQTT() {
  mqttClient.setServer(mqttServer, mqttPort); // Set MQTT broker and port
  mqttClient.setKeepAlive(60);                // Set keep-alive interval to 60 seconds

  Serial.print("Connecting to MQTT broker...");
  if (mqttClient.connect("ESP32Client")) {    // Use a unique client ID
    Serial.println("Connected to MQTT broker!");
      delay(1000); // Allow time for stabilization
                           
    mqttConnected = true;                     // Update the connection state
  } else {
    Serial.print("Failed to connect. MQTT state: ");
    Serial.println(mqttClient.state());
    mqttConnected = false;
  }
}

void publishToMQTT(int people, float frequency) {
  unsigned long now = millis();
  if (mqttClient.connected() && (now - lastPublishTime >= publishInterval)) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      // Format the timestamp
      char timeString[64];
      strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);

      // Construct payload with timestamp
      String payload = String("{\"peopleCount\":") + String(people) +
                       ",\"motionFrequency\":" + String(frequency, 2) +
                       ",\"timestamp\":\"" + timeString + "\"}";

      // Debug the payload before publishing
      Serial.println("Payload before publish: " + payload);

      // Publish the payload
      if (mqttClient.publish(mqttTopic, payload.c_str())) {
        Serial.println("Data published to MQTT: " + payload);
        lastPublishTime = now;
      } else {
        Serial.println("Failed to publish to MQTT. Publish method failed.");
      }
    } else {
      Serial.println("Failed to get current time for MQTT payload.");
    }
  } else if (!mqttClient.connected()) {
    Serial.println("Failed to publish to MQTT. Broker not connected.");
  }
}



// ======== LOG DATA TO CSV ========
// Logs motion data to a CSV file in SPIFFS
// Each log includes a timestamp, people count, frequency, and event type
void logDataToCSV(int people, float frequency, const char* eventType = "Motion") {
  // Open the CSV file in append mode
  File file = SPIFFS.open("/data.csv", FILE_APPEND);
  if (!file) {
    // Error if file cannot be opened for writing
    Serial.println("Failed to open file for appending");
    return;
  }

  // Get the current time for the log entry
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    // Format the timestamp as "YYYY-MM-DD HH:MM:SS"
    char timeString[64];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // Append the data to the CSV file with the given format
    file.printf("%-20s %-10d %-10.2f %-10s\n", timeString, people, frequency, eventType);

    // Debug: Print the logged data to the serial monitor
    Serial.printf("Data logged: %-20s %-10d %-10.2f %-10s\n", timeString, people, frequency, eventType);
  } else {
    // Error if the time could not be retrieved
    Serial.println("Failed to get current time for CSV logging");
  }

  // Close the file after writing
  file.close();
}
// ======== END OF LOG DATA TO CSV ========




// Serve the HTML webpage with live update script
void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>Motion Data</title></head>";
  html += "<body><h1>Motion Detection Data</h1>";
  html += "<p><b>People Count:</b> <span id='peopleCount'>0</span></p>";
  html += "<p><b>Motion Frequency:</b> <span id='motionFrequency'>0</span> events per minute</p>";
  html += "<p><b>Total Motions Detected:</b> <span id='totalMotions'>0</span></p>";
  html += "<p><b>Last Motion Time:</b> <span id='lastMotionTime'>0</span> seconds since boot</p>";
  html += "<button onclick=\"window.location.href='/download'\">Download CSV</button>"; // Download button
  html += "<script>";
  html += "const eventSource = new EventSource('/sse');";
  html += "eventSource.onmessage = function(event) {";
  html += "  const data = JSON.parse(event.data);";
  html += "  document.getElementById('peopleCount').innerText = data.peopleCount;";
  html += "  document.getElementById('motionFrequency').innerText = data.motionFrequency.toFixed(2);";
  html += "  document.getElementById('totalMotions').innerText = data.totalMotions;";
  html += "  document.getElementById('lastMotionTime').innerText = data.lastMotionTime;";
  html += "};";
  html += "</script>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Serve motion data as JSON for live updates
void handleData() {
  float motionFrequency = (float)motionCount / ((millis() - startTime) / 60000.0);
  String json = "{";
  json += "\"peopleCount\":" + String(peopleCount) + ",";
  json += "\"motionFrequency\":" + String(motionFrequency, 2) + ",";
  json += "\"totalMotions\":" + String(totalMotions) + ",";
  json += "\"lastMotionTime\":" + String(lastMotionTime / 1000);
  json += "}";

  server.send(200, "application/json", json);
}

// ======== HANDLE CSV DOWNLOAD AND CLEAR ========
// This function serves the CSV file to the client for download
// and then clears its contents, reinitializing with headers.
void handleDownloadCSV() {
  // Open the CSV file in read mode
  File file = SPIFFS.open("/data.csv", "r");
  if (!file) {
    // Error if file cannot be opened
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  // Serve the file to the client for download
  server.streamFile(file, "text/csv");
  file.close(); // Close the file after serving

  // Open the CSV file in write mode to clear it
  file = SPIFFS.open("/data.csv", FILE_WRITE);
  if (file) {
    // Write back the headers to keep the CSV structure intact
    file.println("Timestamp,PeopleCount,MotionFrequency,EventType");
    file.close();
    Serial.println("CSV file cleared after download."); // Debug message
  } else {
    // Error if the file cannot be cleared
    Serial.println("Failed to clear the CSV file.");
  }

  // Inform the client that the CSV was downloaded and cleared
  server.send(200, "text/plain", "CSV downloaded and cleared.");
}
// ======== END OF CSV DOWNLOAD AND CLEAR ========



// Serve live updates via SSE
void handleSSE() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/event-stream");

  // Prepare JSON data
  float motionFrequency = (float)motionCount / ((millis() - startTime) / 60000.0);
  String json = "{";
  json += "\"peopleCount\":" + String(peopleCount) + ",";
  json += "\"motionFrequency\":" + String(motionFrequency, 2) + ",";
  json += "\"totalMotions\":" + String(totalMotions) + ",";
  json += "\"lastMotionTime\":" + String(lastMotionTime / 1000);
  json += "}";

  // Send data
  server.sendContent("data: " + json + "\n\n");
}

// ======== SETUP FUNCTION ========
// Runs once at startup or after a wake-up event
void setup() {
    // ======== SERIAL COMMUNICATION ========
    Serial.begin(115200); // Start serial communication
    delay(1000); // Short delay for initialization

    // ======== WAKE-UP HANDLING ========
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("Woke up due to motion detected!");
        lastMotionTime = millis(); // Reset motion time after wake-up
    } else {
        Serial.println("Power on or other wake-up cause");
    }

    // ======== FILE SYSTEM (SPIFFS) INITIALIZATION ========
    if (!SPIFFS.begin(true)) {
        Serial.println("An error occurred while mounting SPIFFS");
        return; // Exit setup if SPIFFS fails
    }

    // Initialize the CSV file if it doesn't exist
    File file = SPIFFS.open("/data.csv", FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
    } else {
        if (file.size() == 0) { // Check if the file is empty
            file.println("Timestamp,PeopleCount,MotionFrequency"); // Add headers
            Serial.println("CSV file initialized with headers.");
        }
        file.close(); // Close the file
    }

    // ======== SENSOR PIN SETUP ========
    pinMode(SENSOR_PIN, INPUT); // Set sensor pin as input

    // ======== WIFI CONNECTION ========
    WiFi.begin(ssid, password); // Connect to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("."); // Indicate connection attempt
    }
    Serial.println("\nWi-Fi connected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP()); // Print the device's IP address

    // ======== NTP CONFIGURATION ========
    configTime(3600, 3600, "pool.ntp.org", "time.nist.gov"); // Set up NTP for time synchronization

    // ======== MQTT SETUP ========
    mqttClient.setServer(mqttServer, mqttPort); // Configure MQTT broker
    Serial.print("Connecting to MQTT broker...");
    if (mqttClient.connect("ESP32Client")) { // Use a unique client ID
        Serial.println("Connected to MQTT broker!");
    } else {
        Serial.print("Failed to connect. MQTT state: ");
        Serial.println(mqttClient.state()); // Print the MQTT connection state
    }

    // ======== WEB SERVER SETUP ========
    server.on("/", handleRoot); // Serve the main webpage
    server.on("/data", handleData); // Serve motion data as JSON
    server.on("/download", handleDownloadCSV); // Serve the CSV file for download
    server.on("/sse", handleSSE); // Serve live updates via Server-Sent Events
    server.begin(); // Start the web server
    Serial.println("Web server started!");

    // ======== START TIME INITIALIZATION ========
    startTime = millis(); // Record the start time for motion tracking
}
// ======== END OF SETUP FUNCTION ========

// ======== MAIN LOOP ========
void loop() {
    // ======== MQTT AND HTTP HANDLING ========
    mqttClient.loop(); // Ensure MQTT library handles MQTT connections
    server.handleClient(); // Handle incoming HTTP requests
    ensureMQTTConnection(); // Reconnect to MQTT broker if disconnected

    // ======== SENSOR INPUT DEBOUNCE ========
    int rawState = digitalRead(SENSOR_PIN); // Read the sensor state
    if (rawState != currentDebouncedState) { // Check if the state has changed
        if (millis() - lastDebounceTime > debounceDelay) { // Debounce check
            currentDebouncedState = rawState; // Update the debounced state
            lastDebounceTime = millis(); // Update the last debounce time

            if (currentDebouncedState == LOW) { // Motion detected
                motionActive = true; // Mark motion as active
                lastMotionTime = millis(); // Record the last motion time
                motionCount++; // Increment motion counter
                peopleCount++; // Increment people counter

                // ======== LOG AND PUBLISH INDIVIDUAL MOTION ========
                float frequency = (float)motionCount / ((millis() - startTime) / 60000.0); // Calculate motion frequency
                logDataToCSV(peopleCount, frequency); // Log the motion event
                publishToMQTT(peopleCount, frequency); // Publish motion data to MQTT

                Serial.println("Motion detected and logged."); // Debug message
                Serial.print("Current total: ");
                Serial.println(peopleCount);
            }

            if (currentDebouncedState == HIGH && motionActive) { // Motion stopped
                motionActive = false; // Reset motion state
            }
        }
    }

    // ======== PERIODIC LOGGING ========
    unsigned long currentTime = millis(); // Get the current time
    if (currentTime - lastLogTime >= logInterval) { // Check if it's time to log
        float frequency = (float)motionCount / ((currentTime - startTime) / 60000.0); // Calculate motion frequency
        logDataToCSV(peopleCount, frequency); // Log periodic summary
        publishToMQTT(peopleCount, frequency); // Publish periodic summary to MQTT

        totalMotions += motionCount; // Update total motion count
        intervals++; // Increment log intervals
        motionCount = 0; // Reset motion count after logging
        lastLogTime = currentTime; // Update last log time
    }

    // ======== DEEP SLEEP TRIGGER ========
    if (currentTime - lastMotionTime >= sleepInterval) { // Check if no motion for sleep interval
        Serial.println("No motion for 1 minute. Going to sleep..."); // Debug message
        Serial.print("Total people counted: ");
        Serial.println(peopleCount);

        esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW); // Configure GPIO wake-up
        delay(100); // Short delay before sleep
        esp_deep_sleep_start(); // Enter deep sleep mode
    }

    delay(50); // Short delay for loop timing
}
// ======== END OF MAIN LOOP ========










