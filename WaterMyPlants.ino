#include <BluetoothSerial.h>
#include <BTAddress.h>
#include <BTAdvertisedDevice.h>
#include <BTScan.h>

#include "Adafruit_MAX1704X.h"
Adafruit_MAX17048 maxlipo;

/***************************************************************************
  UPDATED: Tweaked example to work with:
  Adafruit ESP32-S3
  BME680, OLED screen, soil sensor, motorhat, quad rotery dial, 
  MAX17048 voltage monitor, 500 mAh battery, stemma quad hub
  @martywassmer

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

//for oled
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>


#include <Adafruit_MotorShield.h>
#include <math.h>
#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

//quad encoder setup
#define SS_NEO_PIN 18
#define SS_ENC0_SWITCH 12
#define SS_ENC1_SWITCH 14
#define SS_ENC2_SWITCH 17
#define SS_ENC3_SWITCH 9
#define SEESAW_ADDR 0x49
Adafruit_seesaw ss_encoder = Adafruit_seesaw(&Wire);
seesaw_NeoPixel pixels = seesaw_NeoPixel(4, SS_NEO_PIN, NEO_GRB + NEO_KHZ800);
int32_t enc_positions[4] = { 0, 0, 0, 0 };

int loop_delay = 4000;

Adafruit_seesaw ss;
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);  //oled
Adafruit_BME680 bme;                                        // I2C

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
#define BUTTON_A 0
#define BUTTON_B 16
#define BUTTON_C 2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
#define BUTTON_A PA15
#define BUTTON_B PC7
#define BUTTON_C PC5
#elif defined(TEENSYDUINO)
#define BUTTON_A 4
#define BUTTON_B 3
#define BUTTON_C 8
#elif defined(ARDUINO_NRF52832_FEATHER)
#define BUTTON_A 31
#define BUTTON_B 30
#define BUTTON_C 27
#else  // 32u4, M0, M4, nrf52840, esp32-s2, esp32-s3 and 328p
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5
#endif

int sel;
char ip[50] = "None";
int ledState = 0;
int pumpState = 0;

// WiFi network name and password:
const char *networkName = "**";
const char *networkPswd = "**";

// Internet domain to request from:
const char *hostDomain = "google.com";
const int hostPort = 80;

const int LED_PIN = 13;

void setup() {

  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  if (Serial) Serial.println("Setting up display");
  delay(250);                 // wait for the OLED to power up
  display.begin(0x3C, true);  // Address 0x3C default
  display.display();
  display.setRotation(1);
  delay(500);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  if (Serial) Serial.println("Setting up buttons");
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  //check for temp module
  if (!bme.begin()) {
    Serial.println("ERROR! Could not find BME680 sensor");
    //while (1);
  } else {
    Serial.println("Found BME680");
  }

  //check for quad encoder
  if (!ss_encoder.begin(SEESAW_ADDR) || !pixels.begin(SEESAW_ADDR)) {
    Serial.println("ERROR! Quad encoder not found");
    //while(1) delay(1);
  } else {
    Serial.print("Quad encoder started! version: ");
    Serial.println(ss_encoder.getVersion(), HEX);
  }
  ss_encoder.pinMode(SS_ENC0_SWITCH, INPUT_PULLUP);
  ss_encoder.pinMode(SS_ENC1_SWITCH, INPUT_PULLUP);
  ss_encoder.pinMode(SS_ENC2_SWITCH, INPUT_PULLUP);
  ss_encoder.pinMode(SS_ENC3_SWITCH, INPUT_PULLUP);
  ss_encoder.setGPIOInterrupts(1UL << SS_ENC0_SWITCH | 1UL << SS_ENC1_SWITCH | 1UL << SS_ENC2_SWITCH | 1UL << SS_ENC3_SWITCH, 1);

  // get starting positions of rotery
  for (int e = 0; e < 4; e++) {
    enc_positions[e] = ss_encoder.getEncoderPosition(e);
    ss_encoder.enableEncoderInterrupt(e);
  }

  // Initialize all pixel
  pixels.setBrightness(255);
  pixels.setPixelColor(0, Wheel((0 * 4) & 0xFF));
  pixels.setPixelColor(1, Wheel((0 * 4) & 0xFF));
  pixels.setPixelColor(2, Wheel((0 * 4) & 0xFF));
  pixels.setPixelColor(3, Wheel((0 * 4) & 0xFF));
  pixels.show();

  //check for moisture sensor
  if (!ss.begin(0x36)) {
    Serial.println("ERROR! Seesaw not found");
    //while(1) delay(1);
  } else {
    Serial.print("Seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }

  //check for dc motor
  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
                        // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield");
    //while (1);
  } else {
    Serial.print("Motor shield started!");
  }

  //check for voltage monitor
  if (!maxlipo.begin()) {
    Serial.println("Could not find voltage Monitor");
  } else {
    Serial.print("voltage Monitor started!");
  }

  // Connect to the WiFi network (see function below loop)
  // connectToWiFi(networkName, networkPswd);

  //   if(DateTime.available()) { // update clocks if time has been synced
  //    unsigned long prevtime = DateTime.now();
  //   }
}

void sendToDisplay(String message, int delayTime, bool clearDisplay) {
  if (clearDisplay) display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SH110X_WHITE);
  if (clearDisplay) display.setCursor(0, 0);
  display.print(message);
  display.display();  // actually display all of the above
  delay(delayTime);
}

void loop() {

  // Blink led
  digitalWrite(LED_PIN, ledState);
  ledState = (ledState + 1) % 2;  // Flip ledState

  //get data from sensor
  float temp = ((bme.temperature * 180) / 100) + 32;
  float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float pres = bme.pressure / 1000.0;
  float hum = bme.humidity;
  float gas = bme.gas_resistance / 1000.0;

  if (!sel) sel = 0;

  if (Serial) Serial.print("Temperature = ");
  if (Serial) Serial.print(temp);
  if (Serial) Serial.println(" *F");
  // if(Serial)Serial.print("Pressure = ");
  // if(Serial)Serial.print(pres);
  // if(Serial)Serial.println(" kPa");
  // if(Serial)Serial.print("Approx. Altitude = ");
  // if(Serial)Serial.print(alt);
  // if(Serial)Serial.println(" m");
  // if(Serial)Serial.print("Humidity = ");
  // if(Serial)Serial.print(hum);
  // if(Serial)Serial.println(" %");
  // if(Serial)Serial.print("Gas = ");
  // if(Serial)Serial.print(gas);
  // if(Serial)Serial.println(" KOhms");
  if (Serial) Serial.print("IP = ");
  if (Serial) Serial.println(ip);

  //data from soil sensor
  float tempC = ((ss.getTemp() * 180) / 100) + 32;
  uint16_t capread = ss.touchRead(0);
  if (Serial) Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.println("*C");
  if (Serial) Serial.print("Capacitive: ");
  Serial.println(capread);

  //get voltage
  float cellVoltage = maxlipo.cellVoltage();
  if (isnan(cellVoltage)) {
    Serial.println("Failed to read cell voltage, check battery is connected!");
    //delay(2000);
    return;
  } else {
    Serial.print(F("Batt Voltage: "));
    Serial.print(cellVoltage, 3);
    Serial.println(" V");
    Serial.print(F("Batt Percent: "));
    Serial.print(maxlipo.cellPercent(), 1);
    Serial.println(" %");
  }

  if (Serial) Serial.println("-----------------");

  if (!digitalRead(BUTTON_A)) sel = 0;
  if (!digitalRead(BUTTON_B)) sel = 1;
  if (!digitalRead(BUTTON_C)) sel = 2;

  if (sel == 1) {
    myMotor->setSpeed(200);
  }

  //run motor when button pressed
  // while(!digitalRead(BUTTON_A)){
  //     display.clearDisplay();
  //     display.setTextSize(3);
  //     display.setCursor(0,0);
  //     display.print("Running");
  //     display.print("Pump");
  //     if(Serial)Serial.print("Running Pump");
  //     display.display();

  //     myMotor->run(FORWARD);
  //     delay(100);
  // }
  // myMotor->run(RELEASE);



  //check button press on encoders
  if (!ss_encoder.digitalRead(SS_ENC0_SWITCH)) {
    Serial.println("ENC0 pressed!");
  }
  if (!ss_encoder.digitalRead(SS_ENC1_SWITCH)) {
    Serial.println("ENC1 pressed!");
  }
  if (!ss_encoder.digitalRead(SS_ENC2_SWITCH)) {
    Serial.println("ENC2 pressed!");
  }
  if (!ss_encoder.digitalRead(SS_ENC3_SWITCH)) {
    Serial.println("ENC3 pressed!");
  }

  //check encoder positions
  for (int e = 0; e < 4; e++) {
    int32_t new_enc_position = ss_encoder.getEncoderPosition(e);
    // did we move around?
    if (enc_positions[e] != new_enc_position) {
      Serial.print("Encoder #");
      Serial.print(e);
      Serial.print(" -> ");
      Serial.println(new_enc_position);  // display new position

      //ecoder 0 will control delay of loop, keep within range
      if (e == 0) {
        if (new_enc_position < 1) {
          new_enc_position = 0;
          ss_encoder.setEncoderPosition(new_enc_position);
          loop_delay = 40;
        }
        if (new_enc_position > 10) {
          new_enc_position = 10;
          loop_delay = 4000;
          ss_encoder.setEncoderPosition(new_enc_position);
        }
        if (new_enc_position >= 1 and new_enc_position <= 10) {
          loop_delay = (4000 / 10) * (new_enc_position);
        }
        Serial.print("Delay set to ");
        Serial.println(loop_delay);
        enc_positions[e] = new_enc_position;  // save new position


      } else {
        //default behavior
        enc_positions[e] = new_enc_position;  // save new position
      }

      if (e == 1) {
        sel = new_enc_position;
      }

      // change the neopixel color, mulitply the new positiion by 4 to speed it up
      pixels.setPixelColor(e, Wheel((new_enc_position * 10) & 0xFF));
      pixels.show();
    }
  }

  //------------------------------
  //display screens

  display.clearDisplay();

  if (sel == 0) {
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.println("pres:" + String(pres));
    //display.print(pres);
    //display.setCursor(0, 15);
    display.println("hum:" + String(hum));
    //display.print(hum);
    //display.setCursor(0, 30);
    display.println("tmp:" + String(temp));
    //display.print(temp);
    //display.setCursor(0, 45);
    display.println("cap:" + String(capread));
    //display.print(capread);
    display.display();  // actually display all of the above
  }

  if (sel == 1 or maxlipo.cellPercent() <= 10) {

    display.setTextSize(2.0);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.print(cellVoltage, 3);
    display.print(" V");
    display.setCursor(0, 15);
    display.print(maxlipo.cellPercent(), 1);
    display.print(" %");
    display.setCursor(0, 45);
    if (maxlipo.cellPercent() <= 10) {
      display.print("LOW BATTERY");
    }
    display.setCursor(0, 30);
    // we can check if we're hibernating or not
    // if (maxlipo.isHibernating()) {
    //   display.print(F("Hibernating!"));
    // }
    display.print(F("Chg rt:"));
    display.print(maxlipo.chargeRate(), 1);
    display.print(" %/hr");
    display.display();  // actually display all of the above
  }

  if (sel == 2) {
    display.setTextSize(2.2);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.print("IP: ");
    display.print(ip);
    //display.setCursor(0,15);
    display.display();
  }

  if (sel == 3) {
    display.setTextSize(2.0);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.print("Delay: ");
    display.print(loop_delay);
    display.setCursor(0, 15);
    display.print("Scn: ");
    display.print(enc_positions[0]);
    display.setCursor(0, 30);
    for (int e = 0; e < 4; e++) {
      display.print(e);
      display.print(" ");
      display.print(enc_positions[e]);
      display.print(" ");
    }
    display.display();
  }

  if (sel == 4) {
    display.setTextSize(1.5);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.print("Alt: ");
    display.print(alt);
    display.setCursor(0, 15);
    display.print("Gas: ");
    display.print(gas, 2);
    display.print(" KOhms");
    display.display();
  }

  //show blank screen
  if (sel < 0 or sel > 4) {
    display.setTextSize(1.5);
    display.setTextColor(SH110X_WHITE);

    display.setCursor(0, 0);
    display.print("Screen: ");
    display.print(sel);
    display.setCursor(0, 15);
    display.print("BLANK SCREEN");
    display.display();
  }

  delay(loop_delay);
}

void connectToWiFi(const char *ssid, const char *pwd) {
  int ledState = 0;

  //printLine();
  if (Serial) Serial.println("Connecting to WiFi network: " + String(ssid));

  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED) {
    // Blink LED while we're connecting:
    digitalWrite(LED_PIN, ledState);
    ledState = (ledState + 1) % 2;  // Flip ledState
    delay(500);
    Serial.print(".");
  }

  if (Serial) Serial.println();
  if (Serial) Serial.println("WiFi connected!");
  if (Serial) Serial.print("IP address: ");
  if (Serial) Serial.println(WiFi.localIP());
  sprintf(ip, "%s", WiFi.localIP());
}

void requestURL(const char *host, uint8_t port) {
  //printLine();
  if (Serial) Serial.println("Connecting to domain: " + String(host));

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  if (Serial) Serial.println("Connected!");
  //printLine();

  // This will send the request to the server
  client.print((String) "GET / HTTP/1.1\r\n" + "Host: " + String(host) + "\r\n" + "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }

  Serial.println();
  Serial.println("closing connection");
  client.stop();
}

//neo pixel stuff
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return seesaw_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return seesaw_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return seesaw_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
