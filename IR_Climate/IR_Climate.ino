#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Daikin.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Network credentials
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASS";

// Web server on port 80
ESP8266WebServer server(80);

#define OLED_SCL D1 // (GPIO5)
#define OLED_SDA D2 // (GPIO4)
#define OLED_RESET -1 // Reset pin not used on 0.91" OLED

const uint16_t PIN_IRLED = D5;  // (GPIO14)
#define PIN_TEMPERATURE D6      // (GPIO12)
#define PIN_BUZZER D7           // (GPIO13)

#pragma region BUZZER_AUDIO

#define BEEP_FREQ 1600        // Hz
#define GAP_SHORT 250         // ms between beeps in a group
#define GAP_GROUP 600         // ms between groups

// Pattern step encoding: positive => tone ON for that many ms, negative => silence for abs(ms)
typedef struct
{
  const int16_t* steps;
  uint8_t count;
} BeepPattern;

static const int16_t PAT_ONE3_STEPS[] =
{
  100,      // 100 ms tone
  -1        // terminator marker (not used by engine, just for readability)
};

static const uint8_t PAT_ONE3_COUNT = 1;

static const int16_t PAT_SOS_STEPS[] =
{
  // • • •
  100, -GAP_SHORT, 100, -GAP_SHORT, 100, -GAP_GROUP,
  // — — —
  5000, -GAP_SHORT, 500, -GAP_SHORT, 500, -GAP_GROUP,
  // • • •
  100, -GAP_SHORT, 100, -GAP_SHORT, 500
};

static const uint8_t PAT_SOS_COUNT = sizeof(PAT_SOS_STEPS) / sizeof(PAT_SOS_STEPS[0]);

// Engine state
struct
{
  const int16_t* steps = nullptr;
  uint8_t count = 0;
  uint8_t index = 0;
  unsigned long stepStartMs = 0;
  bool active = false;
  bool toneOn = false;
} g_beep;

// Queue flags (set from events / places where you want sounds)
volatile bool g_flagWifiConnected = false;
volatile bool g_flagWifiDisconnected = false;
volatile bool g_flagHttpStarted = false;
volatile bool g_flagHttpStopped = false;
volatile uint8_t g_pendingShortBeeps = 0;   // queued 100 ms beeps (cap applied in enqueueOne3)
static const uint16_t INTER_BEEP_GAP_MS = 120; // silence between chained beeps
static bool g_inGap = false;                // true while we're in that brief silence
static unsigned long g_gapStartMs = 0;      // when the gap started

inline void BeepEngine_start(const BeepPattern& p)
{
  g_beep.steps = p.steps;
  g_beep.count = p.count;
  g_beep.index = 0;
  g_beep.stepStartMs = millis();
  g_beep.active = (p.count > 0);
  g_beep.toneOn = false;
}

inline void BeepEngine_stop()
{
  noTone(PIN_BUZZER);
  g_beep.active = false;
  g_beep.toneOn = false;
}

// Call this from loop()
void BeepEngine_update() 
{
  // Handle inter-beep silent gap BEFORE any early returns.
  if (g_inGap) {
    if (millis() - g_gapStartMs >= INTER_BEEP_GAP_MS) {
      g_inGap = false;
      // Restart the same short beep after the gap
      static const BeepPattern P = { PAT_ONE3_STEPS, PAT_ONE3_COUNT };
      BeepEngine_start(P);
    }
    return; // while in gap, don't process steps
  }

  // Now we can safely early-return when nothing is active.
  if (!g_beep.active || g_beep.steps == nullptr || g_beep.count == 0) return;

  unsigned long now = millis();
  // If just started a step, (re)arm tone/silence
  int16_t dur = g_beep.steps[g_beep.index];
  unsigned long stepDur = (dur >= 0 ? (unsigned long)dur : (unsigned long)(-dur));

  if (now - g_beep.stepStartMs == 0) {
    if (dur > 0) { tone(PIN_BUZZER, BEEP_FREQ); g_beep.toneOn = true; }
    else { noTone(PIN_BUZZER); g_beep.toneOn = false; }
  }

  // Step complete?
  if (now - g_beep.stepStartMs >= stepDur)
  {
    // Advance
    g_beep.index++;
    g_beep.stepStartMs = now;

    if (g_beep.index >= g_beep.count)
    {
      // Pattern done
      BeepEngine_stop();

      // If another short beep is queued, insert a brief silent gap, then replay the same pattern.
      if (g_pendingShortBeeps > 0) 
      {
        g_pendingShortBeeps--;
        g_inGap = true;
        g_gapStartMs = now;
        noTone(PIN_BUZZER); // ensure silence
      }
    } 
    
    else
    {
      // Prepare next step: set/clear tone right away
      int16_t nd = g_beep.steps[g_beep.index];
      if (nd > 0) { tone(PIN_BUZZER, BEEP_FREQ); g_beep.toneOn = true; }
      else { noTone(PIN_BUZZER); g_beep.toneOn = false; }
    }
  }
}

// Helper to enqueue concrete patterns via flags or direct calls
inline void enqueueOne3()
{
  static const BeepPattern P = { PAT_ONE3_STEPS, PAT_ONE3_COUNT };
  
  if (!g_beep.active && !g_inGap) 
  {
    BeepEngine_start(P);
  } 
  
  else 
  {
    // already beeping (or in the short gap) -> queue one more beep
    if (g_pendingShortBeeps < 5) g_pendingShortBeeps++;  // small cap to be safe
  }
}

inline void enqueueSOS()
{
  static const BeepPattern P = { PAT_SOS_STEPS, PAT_SOS_COUNT };
  BeepEngine_start(P);
}

#pragma endregion BUZZER_AUDIO

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DS18B20/DS1820 Temperature sensor setup
OneWire oneWire(PIN_TEMPERATURE);
DallasTemperature sensors(&oneWire);
float currentTemperature = -127.0;
unsigned long lastTempUpdate = 0;
const unsigned long TEMP_UPDATE_INTERVAL = 60000;  // Update every 60 seconds (1 minute)

// AC settings
bool acPower = false;
uint8_t acTemp = 25;  // Default temperature in Celsius
uint8_t acFanSpeed = kDaikinFanAuto;
uint8_t acMode = kDaikinCool;

// Timer settings
unsigned long acTimerStart = 0;
unsigned long acTimerDuration = 0;  // Duration in milliseconds (0 = no timer)

// Initialize the IR sender
IRDaikinESP ac((uint16_t)PIN_IRLED);

// Message for notifications
String notificationMessage = "";

// WiFi event handlers
WiFiEventHandler onGotIPHandler;
WiFiEventHandler onDisconnectedHandler;

// Function to display IP address on OLED
void displayIPAddress() 
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Connected to WiFi");
  display.println(ssid);
  display.println("");
  display.print("IP: ");
  display.println(WiFi.localIP().toString());
  display.display();
}

// Update temperature from the DS1820/DS18B20 sensor
void updateTemperature()
{
  Serial.println("Requesting temperature...");
  sensors.requestTemperatures();
  
  float tempC = sensors.getTempCByIndex(0);
  
  if (tempC != DEVICE_DISCONNECTED_C)
  {
    currentTemperature = tempC;
    Serial.print("Temperature: ");
    Serial.println(currentTemperature);
  } 

  else 
  {
    Serial.println("Error: Could not read temperature data");
  }
  
  lastTempUpdate = millis();
}

// Handle refresh temperature request
void handleRefreshTemp()
{
  updateTemperature();
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle raw temperature data (for debugging)
void handleRawTemp()
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  
  String json = "{\"raw_temp\":";
  json += String(tempC);
  json += ",\"device_status\":";
  json += (tempC == DEVICE_DISCONNECTED_C ? "\"disconnected\"" : "\"connected\"");
  json += ",\"device_count\":";
  json += String(sensors.getDeviceCount());
  json += "}";
  
  server.send(200, "application/json", json);
}

// Set default AC settings
void setDefaultSettings()
{
  ac.setFan(acFanSpeed);
  ac.setMode(acMode);
  ac.setTemp(acTemp);
  ac.setSwingVertical(false);
  ac.setSwingHorizontal(false);
  ac.setQuiet(false);
  ac.setPowerful(false);
}

// Apply current settings and send IR command
void sendAcCommand()
{
  ac.setPower(acPower);
  ac.setTemp(acTemp);
  ac.setFan(acFanSpeed);
  ac.send();
  
  // Debug output
  Serial.println("Sending IR Command:");
  Serial.print("Power: ");
  Serial.println(acPower ? "ON" : "OFF");
  Serial.print("Temperature: ");
  Serial.println(acTemp);
  Serial.print("Mode: ");
  Serial.println(acMode);
  Serial.print("Fan Speed: ");
  Serial.println(acFanSpeed);
}

// Root page handler
void handleRoot()
{
  String html = "<html><head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<title>Daikin AC Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; text-align: center; background-color: black; color: white; }";
  html += "button { background-color: #4CAF50; border: none; color: white; padding: 15px 32px; margin: 10px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; border-radius: 8px; }";
  html += ".off { background-color: #f44336; }";
  html += ".temp { background-color: #008CBA; }";
  html += ".refresh { background-color: #FF9800; }";
  html += ".clear { background-color: #9C27B0; }";
  html += ".sensor { font-size: 1.2em; font-weight: bold; margin: 20px 0; padding: 10px; background-color: #333; border-radius: 5px; color: #fff; }";
  html += ".error { color: #ff6b6b; }";
  html += ".notification { background-color: #2196F3; color: white; padding: 10px; margin: 10px 0; border-radius: 5px; animation: fadeOut 5s forwards; }";
  html += "select { padding: 10px; font-size: 16px; border-radius: 8px; background-color: #008CBA; color: white; margin: 10px; border: none; cursor: pointer; }";
  html += "select:focus { outline: none; }";
  html += ".temp-control, .fan-control, .timer-control { margin: 15px 0; }";
  html += "@keyframes fadeOut { from { opacity: 1; } to { opacity: 0; display: none; } }";
  html += "</style>";
  html += "<script>";
  html += "function refreshTemp() {";
  html += "  fetch('/status')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      const temp = document.getElementById('roomTemp');";
  html += "      if(data.roomTemperature > -100) {";
  html += "        temp.innerText = data.roomTemperature.toFixed(1) + '\\u00B0C';";
  html += "        temp.className = '';";
  html += "      } else {";
  html += "        temp.innerText = 'Sensor Error';";
  html += "        temp.className = 'error';";
  html += "      }";
  html += "      // NEW: live update AC status text from /status";
  html += "      const acStatus = document.getElementById('acStatus');";
  html += "      if (acStatus) acStatus.innerText = data.power ? 'ON' : 'OFF';";
  html += "      const timerStatus = document.getElementById('timerStatus');";
  html += "      if(data.timerActive) {";
  html += "        const total = Math.max(0, Math.floor(data.timerRemaining));";
  html += "        const hours = Math.floor(total / 3600);";
  html += "        const mins = Math.floor((total % 3600) / 60);";
  html += "        const secs = total % 60;";
  html += "        timerStatus.innerHTML = 'Timer: ' + hours + 'h ' + mins + 'm ' + secs + 's remaining';";
  html += "        timerStatus.style.display = 'block';";
  html += "      } else {";
  html += "        timerStatus.style.display = 'none';";
  html += "      }";
  html += "      // Optional safety: ensure timer hidden when power is OFF";
  html += "      if (!data.power && timerStatus) timerStatus.style.display = 'none';";
  html += "    });";
  html += "}";
  html += "function changeTemp() {";
  html += "  const temp = document.getElementById('tempSelect').value;";
  html += "  location.href = '/temp?value=' + temp;";
  html += "}";
  html += "function changeFan() {";
  html += "  const fan = document.getElementById('fanSelect').value;";
  html += "  location.href = '/fan?value=' + fan;";
  html += "}";
  html += "function setTimer() {";
  html += "  const timer = document.getElementById('timerSelect').value;";
  html += "  location.href = '/timer?value=' + timer;";
  html += "}";
  html += "function clearTimer() {";
  html += "  if (confirm('Are you sure you want to clear the timer?')) {";
  html += "    location.href = '/clear_timer';";
  html += "  }";
  html += "}";
  html += "window.onload = function() {";
  html += "  refreshTemp();";
  html += "  setTimeout(function() {";
  html += "    const notification = document.getElementById('notification');";
  html += "    if (notification) notification.style.display = 'none';";
  html += "  }, 5000);";
  html += "};";
  html += "setInterval(refreshTemp, 1000);";
  html += "</script>";
  html += "</head><body>";
  html += "<h1>Daikin AC Control</h1>";
  
  if (notificationMessage.length() > 0) {
    html += "<div id='notification' class='notification'>" + notificationMessage + "</div>";
    notificationMessage = "";
  }
  
  html += "<div class='sensor'>Room Temperature: <span id='roomTemp'>";
  if (currentTemperature > -100)
  {
    html += String(currentTemperature, 1) + "&deg;C";
  } 
  else
  {
    html += "<span class='error'>Sensor Error</span>";
  }
  html += "</span></div>";
  
  html += "<button class='refresh' onclick='location.href=\"/refresh_temp\"'>Refresh Temperature</button><br>";
  html += "<p>AC Status: <span id='acStatus'>" + String(acPower ? "ON" : "OFF") + "</span></p>";
  html += "<p>Set Temperature: " + String(acTemp) + "&deg;C</p>";
  html += "<button onclick='location.href=\"/on\"'>Turn ON</button>";
  html += "<button class='off' onclick='location.href=\"/off\"'>Turn OFF</button><br>";
  
  html += "<div class='temp-control'>";
  html += "<label for='tempSelect'>Select Temperature: </label>";
  html += "<select id='tempSelect' onchange='changeTemp()'>";
  
  for (int t = 18; t <= 28; t++) 
  {
    html += "<option value='" + String(t) + "'";
    if (t == acTemp) html += " selected";
    html += ">" + String(t) + "&deg;C</option>";
  }
  
  html += "</select>";
  html += "</div>";
  
  html += "<div class='fan-control'>";
  html += "<label for='fanSelect'>Select Fan Speed: </label>";
  html += "<select id='fanSelect' onchange='changeFan()'>";
  
  html += "<option value='10'" + String(acFanSpeed == kDaikinFanAuto ? " selected" : "") + ">Auto</option>";
  for (int f = 2; f <= 5; f++) {
    html += "<option value='" + String(f) + "'";
    if (f == acFanSpeed) html += " selected";
    html += ">" + String(f) + "</option>";
  }
  html += "<option value='11'" + String(acFanSpeed == kDaikinFanQuiet ? " selected" : "") + ">Quiet</option>";
  
  html += "</select>";
  html += "</div>";
  
  html += "<div class='timer-control'>";
  html += "<label for='timerSelect'>Auto-Off Timer: </label>";
  html += "<select id='timerSelect' onchange='setTimer()'>";
  
  html += "<option value='0'" + String(acTimerDuration == 0 ? " selected" : "") + ">No Timer</option>";
  for (int h = 1; h <= 4; h++) {
    html += "<option value='" + String(h) + "'";
    if (acTimerDuration == (unsigned long)h * 3600000UL) html += " selected";
    html += ">" + String(h) + " Hour" + (h > 1 ? "s" : "") + "</option>";
  }
  
  html += "</select>";
  
  html += "<button class='clear' onclick='clearTimer()'>Clear Timer</button>";
  
  html += "<p id='timerStatus' style='display: " + String(acTimerDuration > 0 && acPower ? "block" : "none") + ";'>";
  if (acTimerDuration > 0 && acPower) {
    unsigned long elapsed = millis() - acTimerStart;
    unsigned long remainingTime = (elapsed >= acTimerDuration) ? 0UL : (acTimerDuration - elapsed);
    int remainingHours = remainingTime / 3600000UL;
    int remainingMins  = (remainingTime / 60000UL) % 60;
    int remainingSecs  = (remainingTime / 1000UL) % 60;
    html += "Timer: " + String(remainingHours) + "h " + String(remainingMins) + "m " + String(remainingSecs) + "s remaining";
  }
  html += "</p>";
  
  html += "</div>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// Handle power on
void handleOn()
{
  acPower = true;
  
  // Reset timer start time if timer is active
  if (acTimerDuration > 0) 
  {
    acTimerStart = millis();
  }
  
  sendAcCommand();
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle power off
void handleOff()
{
  acPower = false;
  sendAcCommand();
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle temperature changes
void handleTemp()
{
  if (server.hasArg("value"))
  {
    int temp = server.arg("value").toInt();
    if (temp >= 18 && temp <= 30) {  // Typical Daikin temperature range
      acTemp = temp;
      if (acPower) {
        sendAcCommand();
      }
    }
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle fan speed changes
void handleFan()
{
  if (server.hasArg("value")) 
  {
    int fan = server.arg("value").toInt();
    if ((fan >= 2 && fan <= 5) || fan == kDaikinFanAuto || fan == kDaikinFanQuiet) 
    {
      acFanSpeed = fan;
      if (acPower) 
      {
        sendAcCommand();
      }
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle timer changes
void handleTimer()
{
  if (server.hasArg("value")) {
    int hours = server.arg("value").toInt();
    if (hours >= 0 && hours <= 4)
    {
      unsigned long previousTimerDuration = acTimerDuration;
      
      if (hours == 0){
        acTimerDuration = 0; // No timer
      }
      
      else
      {
        acTimerDuration = (unsigned long)hours * 3600000UL; // Convert hours to milliseconds
        if (acPower)
        {
          acTimerStart = millis(); // Start the timer if AC is on
        }
      }
      
      if (previousTimerDuration != acTimerDuration && acPower)
      {
        sendAcCommand();
        String message = "Timer set for " + String(hours) + " hour" + (hours != 1 ? "s" : "");
        notificationMessage = message;
      }
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

// Handle clearing timer
void handleClearTimer()
{
  bool hadActiveTimer = (acTimerDuration > 0);
  
  acTimerDuration = 0;
  
  if (acPower)
  {
    sendAcCommand();
  }
  
  if (hadActiveTimer)
  {
    notificationMessage = "Timer has been cleared";
  }
  
  Serial.println("Timer cleared");
  server.sendHeader("Location", "/");
  server.send(303);
}

// Return current AC status as JSON
void handleStatus()
{
  String json = "{";
  json += "\"power\":" + String(acPower ? "true" : "false") + ",";
  json += "\"temperature\":" + String(acTemp) + ",";
  json += "\"roomTemperature\":" + String(currentTemperature) + ",";
  json += "\"mode\":" + String(acMode) + ",";
  json += "\"fanSpeed\":" + String(acFanSpeed);
  
  if (acTimerDuration > 0 && acPower)
  {
    unsigned long remainingTime = acTimerDuration - (millis() - acTimerStart);
    json += ",\"timerActive\":true";
    json += ",\"timerRemaining\":" + String(remainingTime / 1000UL); // in seconds
  } 

  else 
  {
    json += ",\"timerActive\":false";
    json += ",\"timerRemaining\":0";
  }
  
  json += "}";
  
  server.send(200, "application/json", json);
}

void setup()
{
  // Start serial
  Serial.begin(115200);
  delay(200);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);
  
  // Initialize the IR sender
  ac.begin();
  setDefaultSettings();
  
  // Initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { 
    // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  } 
  
  else 
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Daikin AC Controller");
    display.println("Connecting to WiFi...");
    display.display();
  }
  
  // Initialize the temperature sensors - SIMPLIFIED approach
  Serial.println("Initializing temperature sensor...");
  sensors.begin();
  
  // Get initial temperature reading
  updateTemperature();

  // WiFi event flags (non-blocking)
  onGotIPHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& evt) 
  {
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    g_flagWifiConnected = true;  // handled in loop()
  });

  onDisconnectedHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& evt) 
  {
    Serial.printf("WiFi disconnected, reason=%d\n", evt.reason);
    g_flagWifiDisconnected = true; // handled in loop()
  });
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    // Do NOT beep here; keep association/DHCP fast
  }

  Serial.println("");
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());

  // Guarantee the "connected" beep even if event order differs
  g_flagWifiConnected = true;
  
  // Display IP address on OLED
  displayIPAddress();
  
  // Define API endpoints
  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);
  server.on("/temp", handleTemp);
  server.on("/fan", handleFan);  // New endpoint for fan speed
  server.on("/timer", handleTimer);  // New endpoint for timer
  server.on("/clear_timer", handleClearTimer);  // New endpoint for clearing timer
  server.on("/status", handleStatus);
  server.on("/refresh_temp", handleRefreshTemp);
  server.on("/raw_temp", handleRawTemp);  // Added raw temperature endpoint for debugging

  // Optional endpoint to stop server to test "HTTP stopped" SOS
  server.on("/server_stop", []()
  {
    server.send(200, "text/plain", "Server stopping...");
    delay(50);
    server.stop();
    Serial.println("HTTP server stopped");
    g_flagHttpStopped = true;  // handled in loop()
  });
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
  g_flagHttpStarted = true;   // handled in loop()
}

void loop()
{
  server.handleClient();

  // ---- Non-blocking beeper engine & queued events ----
  if (g_flagWifiConnected) { enqueueOne3(); g_flagWifiConnected = false; }
  if (g_flagWifiDisconnected) { enqueueSOS(); g_flagWifiDisconnected = false; }
  if (g_flagHttpStarted) { enqueueOne3(); g_flagHttpStarted = false; }
  if (g_flagHttpStopped) { enqueueSOS(); g_flagHttpStopped = false; }
  BeepEngine_update();
  // ----------------------------------------------------
  
  // Update temperature reading every minute
  if (millis() - lastTempUpdate >= TEMP_UPDATE_INTERVAL)
  {
    updateTemperature();
  }
  
  // Check if timer is active and has expired
  if (acTimerDuration > 0 && acPower)
  {
    if (millis() - acTimerStart >= acTimerDuration)
    {
      // Timer expired, turn off the AC
      acPower = false;
      acTimerDuration = 0; // Reset timer
      sendAcCommand();
      Serial.println("Timer expired - AC turned off");
    }
  }
}