#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <ir_Daikin.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Message for notifications
String notificationMessage = "";

#pragma region NETWORK_SETTINGS
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASS";
WiFiEventHandler onGotIPHandler;
WiFiEventHandler onDisconnectedHandler;
ESP8266WebServer server(80);
#pragma endregion NETWORK_SETTINGS

#pragma region BUZZER_AUDIO
#define PIN_BUZZER D8           // (GPIO13)
#define BEEP_FREQ 1600          // Hz
#define GAP_SHORT 250           // ms between beeps in a group
#define GAP_GROUP 600           // ms between groups

// Pattern step encoding: positive => tone ON for that many ms, negative => silence for abs(ms)
typedef struct
{
  const int16_t* steps;
  uint8_t count;
} BeepPattern;

static const int16_t BEEP_SUCCESS_STEPS[] =
{
  100,      // 100 ms tone
  -1
};

static const uint8_t BEEP_SUCCESS_COUNT = 1;

static const int16_t BEEP_SOS_STEPS[] =
{
  // • • •
  100, -GAP_SHORT, 100, -GAP_SHORT, 100, -GAP_GROUP,
  // — — —
  500, -GAP_SHORT, 500, -GAP_SHORT, 500, -GAP_GROUP,
  // • • •
  100, -GAP_SHORT, 100, -GAP_SHORT, 500
};

static const uint8_t BEEP_SOS_COUNT = sizeof(BEEP_SOS_STEPS) / sizeof(BEEP_SOS_STEPS[0]);

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

// Queue flags
volatile bool g_flagWifiConnected = false;
volatile bool g_flagWifiDisconnected = false;
volatile bool g_flagHttpStarted = false;
volatile bool g_flagHttpStopped = false;
volatile uint8_t g_pendingShortBeeps = 0;
static const uint16_t INTER_BEEP_GAP_MS = 120;
static bool g_inGap = false;
static unsigned long g_gapStartMs = 0;

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

void BeepEngine_update()
{
  if (g_inGap) {
    if (millis() - g_gapStartMs >= INTER_BEEP_GAP_MS)
    {
      g_inGap = false;
      static const BeepPattern P = { BEEP_SUCCESS_STEPS, BEEP_SUCCESS_COUNT };
      BeepEngine_start(P);
    }

    return;
  }

  if (!g_beep.active || g_beep.steps == nullptr || g_beep.count == 0) return;

  unsigned long now = millis();
  int16_t dur = g_beep.steps[g_beep.index];
  unsigned long stepDur = (dur >= 0 ? (unsigned long)dur : (unsigned long)(-dur));

  if (now - g_beep.stepStartMs == 0)
  {
    if (dur > 0) { tone(PIN_BUZZER, BEEP_FREQ); g_beep.toneOn = true; }
    else { noTone(PIN_BUZZER); g_beep.toneOn = false; }
  }

  if (now - g_beep.stepStartMs >= stepDur)
  {
    g_beep.index++;
    g_beep.stepStartMs = now;

    if (g_beep.index >= g_beep.count)
    {
      BeepEngine_stop();

      if (g_pendingShortBeeps > 0)
      {
        g_pendingShortBeeps--;
        g_inGap = true;
        g_gapStartMs = now;
        noTone(PIN_BUZZER);
      }
    }

    else
    {
      int16_t nd = g_beep.steps[g_beep.index];
      if (nd > 0) { tone(PIN_BUZZER, BEEP_FREQ); g_beep.toneOn = true; }
      else { noTone(PIN_BUZZER); g_beep.toneOn = false; }
    }
  }
}

inline void enqueue_Beep_Success()
{
  static const BeepPattern P = { BEEP_SUCCESS_STEPS, BEEP_SUCCESS_COUNT };
  if (!g_beep.active && !g_inGap) { BeepEngine_start(P); }
  else { if (g_pendingShortBeeps < 5) g_pendingShortBeeps++; }
}

inline void enqueue_Beep_SOS()
{
  static const BeepPattern P = { BEEP_SOS_STEPS, BEEP_SOS_COUNT };
  BeepEngine_start(P);
}
#pragma endregion BUZZER_AUDIO

#pragma region IR__LED
const uint16_t PIN_IRLED = D5;  // (GPIO14)
IRDaikinESP ac((uint16_t)PIN_IRLED);
#pragma endregion IR_LED

#pragma region IR_RECEIVER
const uint16_t kRecvPin = D7;  // D5 on ESP8266
const uint32_t kBaudRate = 115200;
const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;  // Timeout in milliseconds

// Use turn on the save buffer feature for more complete capture coverage.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results;  // Store the results of the IR decoding
#pragma endregion IR_RECEIVER

#pragma region TEMPERATURE_SENSOR
#define PIN_TEMPERATURE D6      // (GPIO12)
OneWire oneWire(PIN_TEMPERATURE);
DallasTemperature sensors(&oneWire);
float currentTemperature = -127.0;
unsigned long lastTempUpdate = 0;
const unsigned long TEMP_UPDATE_INTERVAL = 60000;  // 1 minute
#pragma endregion TEMPERATURE_SENSOR

#pragma region AC_SETTINGS
bool acPower = false;
uint8_t acTemp = 23;
uint8_t acFanSpeed = 3;
uint8_t acMode = kDaikinCool;
unsigned long acTimerStart = 0;
unsigned long acTimerDuration = 0;  // ms (0 = no timer)
#pragma endregion AC_SETTINGS

#pragma region OLED
#define OLED_SCL   D1   // Also known as SCK (GPIO5)
#define OLED_SDA   D2   // (GPIO4)
#define OLED_RESET -1   // not used on most 4-pin modules
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void displayOLED()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  display.println("Connected to WiFi");
  display.println(ssid);
  display.println("");

  display.print("IP: ");
  display.println(WiFi.localIP().toString());
  display.println("");

  display.print("Room Temp: ");
  display.print(currentTemperature);
  display.write(247); // ASCII Degree symbol.
  display.print("C");
  display.println("");

  if (acTimerDuration > 0)
  {
    unsigned long elapsed = millis() - acTimerStart;
    unsigned long remainingTime = (elapsed >= acTimerDuration) ? 0UL : (acTimerDuration - elapsed);
    int remainingHours = remainingTime / 3600000UL;
    int remainingMins  = (remainingTime / 60000UL) % 60;
    String TimeString = String(remainingHours) + "h " + String(remainingMins) + "m ";

    display.println("Remaining Time: ");
    display.print(TimeString);
    display.println("");
  }

  display.display();
}
#pragma endregion OLED

// This function is called every minute. So, we use it to update OLED, too
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
  displayOLED();
}

void handleRefreshTemp()
{
  updateTemperature();
  server.sendHeader("Location", "/");
  server.send(303);
}

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

void sendAcCommand()
{
  ac.setPower(acPower);
  ac.setTemp(acTemp);
  ac.setFan(acFanSpeed);
  ac.send();

  Serial.println("Sending IR Command:");
  Serial.print("Power: "); Serial.println(acPower ? "ON" : "OFF");
  Serial.print("Temperature: "); Serial.println(acTemp);
  Serial.print("Mode: "); Serial.println(acMode);
  Serial.print("Fan Speed: "); Serial.println(acFanSpeed);
}

void handleRoot()
{
  String html = "<html><head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<title>Daikin AC Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; text-align: center; background-color: black; color: white; }";
  html += "button { background-color: #4CAF50; border: none; color: white; padding: 15px 32px; margin: 10px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; border-radius: 8px; cursor: pointer; }";
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
  html += "      const tempEl = document.getElementById('roomTemp');";
  html += "      if (data.roomTemperature > -100) {";
  html += "        tempEl.innerText = data.roomTemperature.toFixed(1) + '\\u00B0C';";
  html += "        tempEl.className = '';";
  html += "      } else {";
  html += "        tempEl.innerText = 'Sensor Error';";
  html += "        tempEl.className = 'error';";
  html += "      }";
  html += "      const acStatus = document.getElementById('acStatus');";
  html += "      if (acStatus) acStatus.innerText = data.power ? 'ON' : 'OFF';";
  html += "      const setTemp = document.getElementById('setTemp');";
  html += "      if (setTemp) setTemp.innerText = String(data.temperature) + '\\u00B0C';";
  html += "      const tempSelect = document.getElementById('tempSelect');";
  html += "      if (tempSelect && tempSelect.value !== String(data.temperature)) {";
  html += "        tempSelect.value = String(data.temperature);";
  html += "      }";
  html += "      const fanSelect = document.getElementById('fanSelect');";
  html += "      if (fanSelect && fanSelect.value !== String(data.fanSpeed)) {";
  html += "        fanSelect.value = String(data.fanSpeed);";
  html += "      }";
  html += "      const timerStatus = document.getElementById('timerStatus');";
  html += "      if (data.timerActive) {";
  html += "        const total = Math.max(0, Math.floor(data.timerRemaining));";
  html += "        const hours = Math.floor(total / 3600);";
  html += "        const mins = Math.floor((total % 3600) / 60);";
  html += "        const secs = total % 60;";
  html += "        timerStatus.innerHTML = 'Timer: ' + hours + 'h ' + mins + 'm ' + secs + 's remaining';";
  html += "        timerStatus.style.display = 'block';";
  html += "      } else {";
  html += "        timerStatus.style.display = 'none';";
  html += "      }";
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
  html += "  location.href = '/clear_timer';";
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
  html += "<p>Set Temperature: <span id='setTemp'>" + String(acTemp) + "&deg;C</span></p>";
  html += "<button onclick='location.href=\"/on\"'>Turn ON</button>";
  html += "<button class='off' onclick='location.href=\"/off\"'>Turn OFF</button><br>";
  html += "<div class='temp-control'>";
  html += "<label for='tempSelect'>Select Temperature: </label>";
  html += "<select id='tempSelect' onchange='changeTemp()'>";
  
  for (int t = 18; t <= 32; t++)
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
  
  for (int f = 1; f <= 5; f++)
  {
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
  
  for (int h = 1; h <= 9; h++)
  {
    html += "<option value='" + String(h) + "'";
    if (acTimerDuration == (unsigned long)h * 3600000UL) html += " selected";
    html += ">" + String(h) + " Hour" + (h > 1 ? "s" : "") + "</option>";
  }

  html += "</select>";

  html += "<p id='timerStatus' style='display: " + String(acTimerDuration > 0 && acPower ? "block" : "none") + ";'>";
  if (acTimerDuration > 0 && acPower)
  {
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

void handleOn()
{
  acPower = true;

  if (acTimerDuration > 0)
  {
    acTimerStart = millis();
  }

  sendAcCommand();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleOff()
{
  acPower = false;
  sendAcCommand();
  server.sendHeader("Location", "/");
  server.send(303);
}

// It is for WebAPIs. Not exposed to web UI.
void handleToggle()
{
  if (acPower == true)
  {
    handleOff();
    return;
  }

  else
  {
    handleOn();
    return;
  }
}

void handleTemp()
{
  if (server.hasArg("value"))
  {
    int temp = server.arg("value").toInt();
    if (temp >= 18 && temp <= 32)
    {
      acTemp = temp;
      if (acPower)
      {
        sendAcCommand();
      }
    }
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleFan()
{
  if (server.hasArg("value"))
  {
    int fan = server.arg("value").toInt();
    if ((fan >= 1 && fan <= 5) || fan == kDaikinFanAuto || fan == kDaikinFanQuiet)
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

void handleClearTimer()
{
  bool hadActiveTimer = (acTimerDuration > 0);

  if (!hadActiveTimer)
  {
    return;
  }

  acTimerDuration = 0;
  notificationMessage = "Timer has been cleared";
  
  if (acPower)
  {
    sendAcCommand();
  }

  displayOLED();
  
  Serial.println("Timer cleared");

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleTimer()
{
  if (server.hasArg("value"))
  {
    int hours = server.arg("value").toInt();
    
    if (hours == 0)
    {
      // It has internal displayOLED and server functions.
      handleClearTimer();
      return;
    }

    else if (hours >= 1 && hours <= 9)
    {
      unsigned long previousTimerDuration = acTimerDuration;

      if (hours == 0)
      {
        acTimerDuration = 0;
      }

      else
      {
        acTimerDuration = (unsigned long)hours * 3600000UL;
        if (acPower)
        {
          acTimerStart = millis();
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
    unsigned long elapsed = millis() - acTimerStart;
    unsigned long remainingTime = (elapsed >= acTimerDuration) ? 0UL : (acTimerDuration - elapsed);
    json += ",\"timerActive\":true";
    json += ",\"timerRemaining\":" + String(remainingTime / 1000UL);
  }

  else
  {
    json += ",\"timerActive\":false";
    json += ",\"timerRemaining\":0";
  }

  json += "}";

  server.send(200, "application/json", json);
}

// Initialize IR receiver
void setupIRReceiver()
{
  irrecv.enableIRIn();  // Start the receiver
  Serial.print("IR Receiver initialized on pin : ");
  Serial.println(kRecvPin);
}

// Display encoding type
void encoding(const decode_type_t protocol)
{
  switch (protocol)
  {
    case DAIKIN:
      Serial.print("Decoded DAIKIN");
      break;
    case DAIKIN2:
      Serial.print("Decoded DAIKIN2");
      break;
    case DAIKIN216:
      Serial.print("Decoded DAIKIN216");
      break;
    case DAIKIN128:
      Serial.print("Decoded DAIKIN128");
      break;
    case DAIKIN152:
      Serial.print("Decoded DAIKIN152");
      break;
    case DAIKIN64:
      Serial.print("Decoded DAIKIN64");
      break;
    default:
      Serial.print("Decoded ");
      Serial.print(typeToString(protocol));
  }
}

// Process the Daikin message to identify power state
void parseDaikin(const decode_results *results)
{
  IRDaikinESP ac(0);
  ac.setRaw(results->state);
  
  // Print the basic protocol information
  Serial.print("Protocol: ");
  Serial.println(typeToString(results->decode_type));
  
  // Print the complete state
  Serial.print("State: ");
  for (uint16_t i = 0; i < results->bits / 8; i++) 
  {
    Serial.printf("%02X", results->state[i]);
  }
  Serial.println("");
  
  // Print power status
  acPower = ac.getPower();
  Serial.print("Power: ");
  Serial.println(acPower ? "ON" : "OFF");

  // Print other settings if available
  if (results->decode_type == DAIKIN)
  {
    acMode = ac.getMode();
    Serial.print("Mode: ");
    switch(acMode)
    {
      case kDaikinAuto: Serial.println("Auto"); break;
      case kDaikinCool: Serial.println("Cool"); break;
      case kDaikinHeat: Serial.println("Heat"); break;
      case kDaikinDry: Serial.println("Dry"); break;
      case kDaikinFan: Serial.println("Fan"); break;
      default: Serial.println("Unknown");
    }
    
    const float temporary_ac_temp = ac.getTemp();
    acTemp = int(trunc(temporary_ac_temp));
    Serial.print("Temperature: ");
    Serial.println(temporary_ac_temp);

    acFanSpeed = ac.getFan();
    Serial.print("Fan Speed: ");
    Serial.println(ac.getFan());

    Serial.print("Is Off Timer Enabled: ");
    Serial.println(ac.getOffTimerEnabled() ? "YES" : "NO");

    // If printed value is 1536, it means timer is off.
    Serial.print("Off Time: ");
    Serial.println(ac.getOffTime());

    Serial.print("Is On Timer Enabled: ");
    Serial.println(ac.getOnTimerEnabled() ? "YES" : "NO");

    Serial.print("On Time: ");
    Serial.println(ac.getOnTime());

    // Refresh Web UI.
    server.sendHeader("Location", "/");
    server.send(303);
  }
  
  Serial.println("----------------------------------------");
}

void processIrMessage()
{
  if (irrecv.decode(&results))
    {
      // Display basic information
      Serial.println();
      Serial.print("IR Signal Received at ");
      Serial.println(millis());
      encoding(results.decode_type);
      Serial.print(" (");
      Serial.print(results.bits, DEC);
      Serial.println(" bits)");

      // If it's a Daikin protocol, process it in detail
      if (results.decode_type == DAIKIN || results.decode_type == DAIKIN2 || results.decode_type == DAIKIN216 || results.decode_type == DAIKIN128 || results.decode_type == DAIKIN152 || results.decode_type == DAIKIN64)
      {
        parseDaikin(&results);
      } 
      
      else 
      {
        // For non-Daikin signals, show the raw data
        Serial.println("Raw data (probably not Daikin):");
        Serial.println(resultToSourceCode(&results));
      }
      
      irrecv.resume();
    }
}

void setup()
{
  Serial.begin(115200);
  delay(200);

#pragma region SETUP_OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);                 // keep 100 kHz while testing stability
  if (!display.begin(0x3C, true))        // SH1106: (i2c_addr, reset)
  {
    Serial.println(F("SH1106 init failed"));
  }

  else
  {
    display.setRotation(0);              // many 1.3" boards prefer 2; try 0/1/3 if needed
    display.setContrast(0x2F);           // some clones behave better with lower contrast
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("Daikin AC Controller");
    display.println("Connecting to WiFi...");
    display.display();
  }
#pragma endregion SETUP_OLED

#pragma region SETUP_BUZZER
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);
#pragma endregion SETUP_BUZZER

#pragma region SETUP_AC
  ac.begin();
  setDefaultSettings();
#pragma endregion SETUP_AC

#pragma region SETUP_WIFI
  onGotIPHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& evt)
  {
    //Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    g_flagWifiConnected = true;
  });

  onDisconnectedHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& evt)
  {
    String WiFi_Fail_Message = "WiFi disconnected, reason=" + String(evt.reason);
    Serial.println(WiFi_Fail_Message);
    g_flagWifiDisconnected = true;

    display.setRotation(0);              // many 1.3" boards prefer 2; try 0/1/3 if needed
    display.setContrast(0x2F);           // some clones behave better with lower contrast
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("Daikin AC Controller");
    display.println(WiFi_Fail_Message);
    display.display();
  });

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to WiFi: ");
  Serial.println(ssid);

  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
#pragma endregion SETUP_WIFI

#pragma region SETUP_DS1820
  Serial.println("Initializing temperature sensor...");
  sensors.begin();
  
  // it has internal displayOLED() function.
  updateTemperature();
#pragma endregion SETUP_DS1820

#pragma region SETUP_HTTP
  server.on("/", handleRoot);
  server.on("/on", handleOn);
  server.on("/off", handleOff);
  server.on("/toggle", handleToggle);
  server.on("/temp", handleTemp);
  server.on("/fan", handleFan);
  server.on("/timer", handleTimer);
  server.on("/clear_timer", handleClearTimer);
  server.on("/status", handleStatus);
  server.on("/refresh_temp", handleRefreshTemp);

  server.begin();
  Serial.println("HTTP server started");
  g_flagHttpStarted = true;
#pragma endregion SETUP_HTTP

  setupIRReceiver();
}

void loop()
{
  server.handleClient();

  if (g_flagWifiConnected)   { enqueue_Beep_Success(); g_flagWifiConnected = false; }
  if (g_flagWifiDisconnected){ enqueue_Beep_SOS();  g_flagWifiDisconnected = false; }
  if (g_flagHttpStarted)     { enqueue_Beep_Success(); g_flagHttpStarted = false; }
  if (g_flagHttpStopped)     { enqueue_Beep_SOS();  g_flagHttpStopped = false; }
  BeepEngine_update();

  if (millis() - lastTempUpdate >= TEMP_UPDATE_INTERVAL)
  {
    updateTemperature();
  }

  if (acTimerDuration > 0 && acPower)
  {
    if (millis() - acTimerStart >= acTimerDuration)
    {
      acPower = false;
      acTimerDuration = 0;
      sendAcCommand();
      Serial.println("Timer expired - AC turned off");
    }
  }

  processIrMessage();
}