# ESP8266_Daikin_Controller
ESP8266 based Daikin Climate IR Remote Controller and Dallas DS1820 Temprature Sensor.

## FEATURES
- Turn on and off AC.
- Change temperature
- Get room temperature with sensor (it refreshes at 60 seconds)
- It has a web panel and API.

## LIBRARIES
You can install them from Arduino IDE libraries section.
- IRremoteESP8266 : https://github.com/crankyoldgit/IRremoteESP8266
- (Optional for temperature sensor) DallasTemperature : https://github.com/milesburton/Arduino-Temperature-Control-Library
- (Optional for temperature sensor) OneWire : https://www.pjrc.com/teensy/td_libs_OneWire.html
- (Optional for OLED) Adafruit GFX Lbrary
- (Optional for OLED) Adafruit SSD1305 / 1306

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
- 1X ESP8266 NodeMCU
- 1X 5mm 940nm 80 mW IR LED (Red LED in scheme) (Pins: Long leg is (+) and short leg s (- / GRD)
- 1X S8050 Transistor (Transistor with N sign in scheme) (Pins: Emitter - Base - Collector)
- 1X 4.7K ohm Resistor
- 1X Breadboard (I used big one)
- (Optional) 1X Dallas DS1820 Temperature Sensor (Pins: GND - DQ - VDD)
- (Optional) 1X Female barrel jack for 5V input to ESP8266 for easy usage.
- (Optional) 0.91 OLED Display to print connected WiFi and LAN IP Address.

## INFO ABOUT PHOTO.JPG
I didn't have correct colored male to male cables for connection. So, they are mixed. In the future I can consider to connect them on perfboard.
