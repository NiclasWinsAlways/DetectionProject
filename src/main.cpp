#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include "esp_sleep.h"
#include <PubSubClient.h>


// Wi-Fi credentials
const char* ssid = "E308";
const char* password = "98806829";

// GPIO pin for FC-51 sensor
#define SENSOR_PIN 33

// Web server on port 80
WebServer server(80);

// Variables saved during deep sleep
RTC_DATA_ATTR unsigned long lastMotionTime = 0; 
RTC_DATA_ATTR int motionCount = 0; 
RTC_DATA_ATTR int peopleCount = 0; 
RTC_DATA_ATTR unsigned long startTime = 0; 

// Additional tracking variables
static int totalMotions = 0; 
static int intervals = 0; 

// Timers
unsigned long lastLogTime = 0;                 // Tracks the last time data was logged
const unsigned long logInterval = 10000;      // Log data every 10 seconds
const unsigned long sleepInterval = 60000;    // Sleep after 1 minute of no motion
const unsigned long debounceDelay = 200; // Debounce time (ms)
unsigned long lastReconnectAttempt = 0; // Tracks the last reconnection attempt time
const unsigned long reconnectInterval = 60000; // 1 minute interval (in milliseconds)
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 1000; // Publish at most once per second  




unsigned long lastDebounceTime = 0; 
int previousState = HIGH; 
int currentDebouncedState = HIGH; 
bool motionActive = false;
// Tracks the MQTT connection state
bool mqttConnected = false; 


// MQTT broker details
const char* mqttServer = "test.mosquitto.org"; // Public test broker
const int mqttPort = 1883;
const char* mqttTopic = "motion/data";

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



// Log data to CSV
void logDataToCSV(int people, float frequency, const char* eventType = "Motion") {
  File file = SPIFFS.open("/data.csv", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeString[64];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // Log with EventType
    file.printf("%-20s %-10d %-10.2f %-10s\n", timeString, people, frequency, eventType);

    Serial.printf("Data logged: %-20s %-10d %-10.2f %-10s\n", timeString, people, frequency, eventType);
  } else {
    Serial.println("Failed to get current time for CSV logging");
  }
  file.close();
}



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

// Serve the CSV file for download
void handleDownloadCSV() {
  File file = SPIFFS.open("/data.csv", "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  // Serve the file for download
  server.streamFile(file, "text/csv");
  file.close();

  // Clear the file immediately after serving
  file = SPIFFS.open("/data.csv", FILE_WRITE); // Open in write mode to overwrite
  if (file) {
    file.println("Timestamp,PeopleCount,MotionFrequency,EventType"); // Reinitialize with headers
    file.close();
    Serial.println("CSV file cleared after download.");
  } else {
    Serial.println("Failed to clear the CSV file.");
  }
}



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

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woke up due to motion detected!");
    lastMotionTime = millis(); // Reset motion time after wake-up
  } else {
    Serial.println("Power on or other wake-up cause");
  }

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error occurred while mounting SPIFFS");
    return;
  }

  File file = SPIFFS.open("/data.csv", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
  } else {
    if (file.size() == 0) {
      file.println("Timestamp,PeopleCount,MotionFrequency");
      Serial.println("CSV file initialized with headers.");
    }
    file.close();
  }

  pinMode(SENSOR_PIN, INPUT);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Configure NTP
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");

  // Set up MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  Serial.print("Connecting to MQTT broker...");
  if (mqttClient.connect("ESP32Client")) { // Use a unique client ID
    Serial.println("Connected to MQTT broker!");
  } else {
    Serial.print("Failed to connect. MQTT state: ");
    Serial.println(mqttClient.state()); // Print the MQTT state
  }

  // Set up the web server
  server.on("/", handleRoot); // Serve the webpage
  server.on("/data", handleData); // Serve motion data as JSON
  server.on("/download", handleDownloadCSV); // Serve the CSV file
  server.on("/sse", handleSSE); // Serve live updates
  server.begin();
  Serial.println("Web server started!");

  // Set the start time for wake-up grace period
  startTime = millis();
}



void loop() {
    mqttClient.loop(); // Ensure MQTT library handles connection
    server.handleClient(); // Handle HTTP requests
    ensureMQTTConnection();

    int rawState = digitalRead(SENSOR_PIN);

    // Debounce logic
    if (rawState != currentDebouncedState) {
        if (millis() - lastDebounceTime > debounceDelay) {
            currentDebouncedState = rawState;
            lastDebounceTime = millis();

            if (currentDebouncedState == LOW) { // Motion detected
                motionActive = true;
                lastMotionTime = millis();
                motionCount++;
                peopleCount++;

                // Log individual motion event
                float frequency = (float)motionCount / ((millis() - startTime) / 60000.0);
                logDataToCSV(peopleCount, frequency); // Log motion immediately

                // Use the publishToMQTT function to publish data
                publishToMQTT(peopleCount, frequency);

                Serial.println("Motion detected and logged.");
                Serial.print("Current total: ");
                Serial.println(peopleCount);
            }

            if (currentDebouncedState == HIGH && motionActive) {
                motionActive = false;
            }
        }
    }

    unsigned long currentTime = millis();

    // Periodic log every 10 seconds
    if (currentTime - lastLogTime >= logInterval) {
        float frequency = (float)motionCount / ((currentTime - startTime) / 60000.0);
        logDataToCSV(peopleCount, frequency); // Periodic log for summary

        // Use the publishToMQTT function for periodic log
        publishToMQTT(peopleCount, frequency);

        totalMotions += motionCount;
        intervals++;
        motionCount = 0; // Reset after logging
        lastLogTime = currentTime; // Update log timer
    }

    // Go to sleep after 1 minute of no motion
    if (currentTime - lastMotionTime >= sleepInterval) {
        Serial.println("No motion for 1 minute. Going to sleep...");
        Serial.print("Total people counted: ");
        Serial.println(peopleCount);

        esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, LOW); // Enable GPIO wake-up
        delay(100);
        esp_deep_sleep_start();
    }

    delay(50);
}









