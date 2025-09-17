# ESP8266_Daikin_Controller
ESP8266 based Daikin AC Controller.

## FEATURES
- Turn on and off AC.
- Change temperature.
- Change fan speed.
- Set, change or clear auto off timer.
- It synchronized with original remote controller. (When you change something from remote controller, it will apply to ESP8266. But you can't see ESP8266 based changes on remote controller because it doesn't have an IR receiver. It just holds configurations.)
- Get room temperature with sensor (it refreshes at every 60 seconds).
- It has a web panel and API.

## LIBRARIES
You can install them from Arduino IDE libraries section.
- IRremoteESP8266 : https://github.com/crankyoldgit/IRremoteESP8266
- (Optional for temperature sensor) DallasTemperature : https://github.com/milesburton/Arduino-Temperature-Control-Library
- (Optional for temperature sensor) OneWire : https://www.pjrc.com/teensy/td_libs_OneWire.html
- (Optional for OLED) Adafruit GFX Lbrary
- (Optional for OLED) Adafruit SSD1305 / 1306 / SH110X

## IDE PLUGINS (OPTINAL IF YOU WANT TO STORE HTML AS A FILE)
- LittleFS Uploader : https://github.com/earlephilhower/arduino-littlefs-upload/releases/tag/1.5.4

## LittleFS USAGE (OPTINAL IF YOU WANT TO STORE HTML AS A FILE)
- Close Arduino IDE
- Move downloaded .vsix file to ``C:\Users\USERNAME\.arduinoIDE\plugins`` (not "plugin-storage". you can create "plugins" folder if you don't have already.)
- Open Arduino IDE
- Press ``CTRL + SHIFT + P`` buttons to open plugins list.
- Search LittleFS.
- Your files should be in ``SketchFolder/data``. This path is constant. Don't change it. If you delete something from there and do upload, it will remove that files.

## ARDUINO IDE ESP8266 INSTALLATION
- Add this to ``Arduino IDE / File / Preferences / Additional Boards Manager URLs`` ``http://arduino.esp8266.com/stable/package_esp8266com_index.json``
- Install ESP8266 Boards from ``Arduino IDE / Tools / Boards Manager / Search "esp8266 by ESP8266 Community"``
- Select ``NodeMCU MCU 1.0 (ESP-12E Module) as target device.``
- If Windows 11 can't see ESP8266 correctly in ``Device Manager``, you have to install this driver.
Unzip driver, right click device, select update driver, choose driver folder. </br> https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads

## HARDWARES
- 1X ESP8266 NodeMCU (ESP32 doesn't work with IRremoteESP8266. You can track its support from its issues)
- 1X 5mm 940nm 80 mW IR LED (Blue LED in scheme) (Pins: Long leg is (+) and short leg s (- / GRD)
- 1X S8050 Transistor (Transistor with N sign in scheme) (Pins: Emitter - Base - Collector)
- 1X 4.7K ohm Resistor
- 1X Breadboard (I used big one)
- 1X IR Receiver to sync. with original remote controller.
- (Optional) 1X Dallas DS1820 Temperature Sensor (Pins: GND - DQ - VDD)
- (Optional) OLED Display to print connected WiFi and LAN IP Address.
- (Optional) 1X Female barrel jack or Breadboard Power Supply Module for easy power delivery. You know, ESP8266'e USB input requires 3.3V input and finding that adapter can be hard. This allow me to use 5V adapters. I used power supply module.

## INFO ABOUT PHOTOGRAPH
Connection scheme is same with Fritzing but I did some cable management and used power module for cleaner product.

## UI Preview
index.html file and Github Pages show what UI will look like when you install this .ino file to your ESP8266. Preview file generated from GitHub Copilot. So, there can be some little differences. Auto-Off Timer Selection Box will open when AC is on.

## ROADMAP
HiveMQ based MQTT for remote access. (I have CGN IP and I need to deploy a tunnelling solution to access it from outside of my local network. Connecting HiveMQ or similar (free and serverless) MQTT broker service and developing an app to controll it will solve remote access problem.
