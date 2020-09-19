//#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <esp_now.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include "images.h"
#include "SH1106Wire.h"
#include "secret.h"
#include "OLEDDisplay.h"
#include "settings.h"
#include <DHT.h>
#include <PZEM004Tv30.h>
DHT dht(DHTPIN, DHTTYPE);
PZEM004Tv30 pzem(&Serial2);

BlynkTimer timer;
bool ONLINE = false;
bool AUTO_MODE = true;
bool PUMP_ON = false;
bool LED_ON = true;
bool DISPLAY_ON = true;
bool SOUND_ON = true;
bool GAS_LEAK = false;
SH1106Wire display(0x3c, SDA, SCL);
int tank_percent = 0;
int sump_percent = 0;
int sensor_distance = 0;
int sensor_thresold = 5;
int sump_level = 0;
float voltage_blynk, current_blynk, power_blynk;
const unsigned long autoFillInterval = 600000L;

typedef struct struct_message
{
  long d;
  long stamp;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
  sensor_distance = myData.d;
  if (ONLINE)
    Blynk.virtualWrite(vPIN_ESPNOW, myData.stamp);
  //Serial.println(myData.d);
  //Serial.println(myData.stamp);
  //Serial.println();
}

void setup()
{
  // Debug console
  //Serial.begin(115200);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  pinMode(GAS_DO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);
  pinMode(RELAY_PIN_1, OUTPUT);
  digitalWrite(RELAY_PIN_1, LOW);
  pinMode(RELAY_PIN_2, OUTPUT);
  digitalWrite(RELAY_PIN_2, HIGH);

  display.init();
  display.clear();
  display.flipScreenVertically();
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, 4);
  //  while (WiFi.status() != WL_CONNECTED) {
  //    delay(1000);
  //    Serial.println("Setting as a Wi-Fi Station..");
  //  }
  //Serial.print("Station IP Address: ");
  //Serial.println(WiFi.localIP());
  //Serial.print("Wi-Fi Channel: ");
  //Serial.println(WiFi.channel());
  Blynk.config(AUTH);
  if (esp_now_init() != ESP_OK)
  {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  dht.begin();

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  timer.setInterval(30000L, checkBlynkStatus);
  timer.setInterval(2000L, getSumpLevel);
  timer.setInterval(2000L, getTankLevel);
  timer.setInterval(4000L, getDHTSensor);
  timer.setInterval(5000L, detectLPG);
  timer.setInterval(2000L, get_pzem_data);
  timer.setInterval(1000L, runDisplay);
  timer.setInterval(500L, TouchReadPump);
  timer.setInterval(500L, TouchReadAuto);
  timer.setInterval(500L, TouchReadLights);
  timer.setInterval(autoFillInterval, autonomousFill);
}

BLYNK_CONNECTED()
{
  Blynk.virtualWrite(vPIN_AUTO_MODE, AUTO_MODE);
  Blynk.virtualWrite(vPIN_DISPLAY, HIGH);
  Blynk.virtualWrite(vPIN_12VLED, LED_ON);
  Blynk.virtualWrite(vPIN_PUMP, PUMP_ON);
}

BLYNK_WRITE(vPIN_PUMP)
{
  int buttonState = param.asInt(); //pinData variable will store value that came via Bridge
  if (buttonState == 0)
    PUMP_ON = false;
  else
    PUMP_ON = true;
  digitalWrite(RELAY_PIN_1, buttonState);
}

BLYNK_WRITE(vPIN_AUTO_MODE)
{
  int buttonState = param.asInt();
  if (buttonState == 0)
    AUTO_MODE = false;
  else
    AUTO_MODE = true;
}

BLYNK_WRITE(vPIN_SOUND)
{
  int buttonState = param.asInt(); //pinData variable will store value that came via Bridge
  if (buttonState == 0)
    SOUND_ON = false;
  else
    SOUND_ON = true;
}

BLYNK_WRITE(vPIN_12VLED)
{
  int buttonState = param.asInt();
  digitalWrite(RELAY_PIN_2, buttonState);
}

BLYNK_WRITE(vPIN_BUZZER)
{
  int buttonState = param.asInt();
  digitalWrite(BUZZER_PIN, buttonState); //LOW activate
}

BLYNK_WRITE(vPIN_DISPLAY)
{
  int buttonState = param.asInt();
  if (buttonState == LOW)
    display.displayOff();
  else
    display.displayOn();
}

void loop()
{
  //  if (WiFi.status() == WL_CONNECTED)
  //  {
  //    //Serial.println(" Connected");
  //    Blynk.run();
  //  }
  //  timer.run();

  //  ONLINE = Blynk.connected();
  if (ONLINE)
  {
    if (Blynk.connected())
      Blynk.run();
    else
      ONLINE = false;
  }
  timer.run();
}

void checkBlynkStatus()
{
  if (!ONLINE)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (Blynk.connected())
        ONLINE = true;
      else
      {
        Blynk.disconnect();
        ONLINE = Blynk.connect();
        //delay(100);
      }
    }
    else
    {
      ESP.restart();
    }
  }
}

void runDisplay()
{
  display.clear();
  drawTankBar();
  drawSumpBar();
  drawTankPercent();
  drawStatusIcons();
  display.display();
}

void drawTankBar()
{
  // display.setColor(WHITE);
  // display.drawRect(88, 0, 40, 60);
  // int fill = map(tank_percent, 0, 100, 70, 0);
  // // Fill the rectangle
  // display.fillRect(88, fill, 40, 60);
  display.setColor(WHITE);
  display.drawRect(65, 0, 30, 64);
  int fill = map(tank_percent, 0, 100, 65, 0);
  display.fillRect(65, fill, 30, 64);
}

void drawSumpBar()
{
  display.setColor(WHITE);
  display.drawRect(100, 0, 28, 64);
  int sumpfill = map(sump_percent, 0, 100, 65, 0);
  display.fillRect(100, sumpfill, 28, 64);
}

void drawStatusIcons()
{
  if (ONLINE)
    display.setColor(WHITE);
  else
    display.setColor(BLACK);
  display.drawXbm(0, 48, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);

  if (AUTO_MODE)
    display.setColor(WHITE);
  else
    display.setColor(BLACK);
  display.drawRect(22, 48, 36, 15);
  display.setFont(ArialMT_Plain_10);
  display.drawString(26, 49, "AUTO");
}

void animateTankBar()
{
}

void drawTankPercent()
{
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(DialogInput_plain_30);
  display.setColor(WHITE);
  if (tank_percent >= 100)
    display.setFont(DialogInput_plain_25);
  String percent = String(tank_percent);
  percent += "%";
  display.drawString(0, 0, percent);
}

void getSumpLevel()
{
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIGPIN, LOW);

  long duration = pulseIn(ECHOPIN, HIGH);
  sump_level = duration / 58.2;
  int sumpFilllevel = SUMP_HEIGHT_IN_CM - sump_level;
  if (sump_level >= WATER_TANK_HEIGHT_IN_CM)
  {
    sump_percent = 0;
  }
  else
    sump_percent = (((float)sumpFilllevel / SUMP_HEIGHT_IN_CM) * 100);

  if (ONLINE)
  {
    Blynk.virtualWrite(vPIN_SUMP_PERCENT, sump_percent);
    Blynk.virtualWrite(vPIN_SUMP_LEVEL, sump_level);
  }
  if (sumpFilllevel <= SUMP_MIN_LEVEL && PUMP_ON)
  {
    if (AUTO_MODE)
    {
      turnOffPump();
      if (ONLINE)
        Blynk.notify("NO Water in sump. Pump is turned OFF.");
    }
  }
}

void getTankLevel()
{
  int waterFilllevel = WATER_TANK_HEIGHT_IN_CM - sensor_distance;
  if (sensor_distance >= WATER_TANK_HEIGHT_IN_CM)
  {
    tank_percent = 0;
  }
  if (sensor_distance < sensor_thresold && sensor_distance > 0)
  {
    tank_percent = 100;
  }
  if (sensor_distance >= sensor_thresold && sensor_distance <= WATER_TANK_HEIGHT_IN_CM)
  {
    tank_percent = (((float)waterFilllevel / FULL_TANK_LEVEL) * 100);
  }
  if (ONLINE)
  {
    Blynk.virtualWrite(vPIN_TANK_PERCENT, tank_percent);
    Blynk.virtualWrite(vPIN_TANK_LEVEL, sensor_distance);
  }
  if (sensor_distance <= sensor_thresold && PUMP_ON)
  {
    if (AUTO_MODE)
    {
      turnOffPump();
      if (ONLINE)
        Blynk.notify("Tank FULL 100%. Pump is turned OFF.");
    }
  }
}

void turnOffPump()
{
  PUMP_ON = false;
  digitalWrite(RELAY_PIN_1, LOW);
  if (ONLINE)
    Blynk.virtualWrite(vPIN_PUMP, LOW);
  // digitalWrite(RELAY_PIN_1, 0);
  // digitalWrite(D4, 0);
}
void turnONPump()
{
  PUMP_ON = true;
  digitalWrite(RELAY_PIN_1, HIGH);
  if (ONLINE)
    Blynk.virtualWrite(vPIN_PUMP, HIGH);
  // digitalWrite(RELAY_PIN_1, 0);
  // digitalWrite(D4, 0);
}

void detectLPG()
{
  int LPG_Status = digitalRead(GAS_DO);
  digitalWrite(BUZZER_PIN, LPG_Status);
  if (ONLINE)
  {
    Blynk.virtualWrite(vPIN_LPG, LPG_Status);
    if (LPG_Status == LOW)
      Blynk.notify("GAS LEAK in Kitchen!");
  }
}

void getDHTSensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t))
  {
    //Serial.println("Failed to read from DHT sensor!");
    return;
  }
  if (ONLINE)
  {
    Blynk.virtualWrite(vPIN_TEMPERATURE, t);
    Blynk.virtualWrite(vPIN_HUMIDITY, h);
  }
}

void get_pzem_data()
{
  if (!isnan(pzem.voltage()))
    voltage_blynk = pzem.voltage();
  if (!isnan(pzem.current()))
    current_blynk = pzem.current();
  if (!isnan(pzem.power()))
    power_blynk = pzem.power();

  if (ONLINE)
  {
    Blynk.virtualWrite(vPIN_SIGNAL, WiFi.RSSI());
    Blynk.virtualWrite(vPIN_VOLTAGE, voltage_blynk);
    Blynk.virtualWrite(vPIN_CURRENT_USAGE, current_blynk);
    Blynk.virtualWrite(vPIN_ACTIVE_POWER, power_blynk);
  }
}

void TouchButton()
{
  int touch = touchRead(T4);
  //Serial.println("touch:");
  //Serial.println(touch);
  if (touch<30 & touch> 10)
  {
    //Serial.println("touch:");
    //Serial.println(touch);
    int state = digitalRead(RELAY_PIN_1);
    state = !state;
    digitalWrite(RELAY_PIN_1, state);
  }
}

void TouchReadPump()
{
  static int positiveTouchCounter = 0;
  static int positiveTouchCounterLimit = 3;
  int touchReadValue = touchRead(T4);
  //Serial.println("touch:");
  //Serial.println(touchReadValue);
  if (touchReadValue < 20)
  {
    positiveTouchCounter++;
    //delay(10);
  }
  else
  {
    positiveTouchCounter = 0;
  }

  if (positiveTouchCounter >= positiveTouchCounterLimit)
  {
    //ShelfLightState();
    playBeep();
    PUMP_ON = !PUMP_ON;
    int state = digitalRead(RELAY_PIN_1);
    state = !state;
    digitalWrite(RELAY_PIN_1, state);
    if (ONLINE)
      Blynk.virtualWrite(vPIN_PUMP, state);
    positiveTouchCounter = 0;
    //delay(1000);
  }
}

void TouchReadLights()
{
  static int positiveTouchCounter = 0;
  static int positiveTouchCounterLimit = 3;
  int touchReadValue = touchRead(T6);
  //Serial.println("touch:");
  //Serial.println(touchReadValue);
  if (touchReadValue < 20)
  {
    positiveTouchCounter++;
    //delay(10);
  }
  else
  {
    positiveTouchCounter = 0;
  }

  if (positiveTouchCounter >= positiveTouchCounterLimit)
  {
    //ShelfLightState();
    //AUTO_MODE = !AUTO_MODE;
    playBeep();
    LED_ON = !LED_ON;
    int state = digitalRead(RELAY_PIN_2);
    state = !state;
    digitalWrite(RELAY_PIN_2, state);
    if (ONLINE)
      Blynk.virtualWrite(vPIN_12VLED, state);
    positiveTouchCounter = 0;
    //delay(1000);
  }
}

void TouchReadAuto()
{
  static int positiveTouchCounter = 0;
  static int positiveTouchCounterLimit = 3;
  int touchReadValue = touchRead(T5);
  //Serial.println("touch:");
  //Serial.println(touchReadValue);
  if (touchReadValue < 20)
  {
    positiveTouchCounter++;
    //delay(10);
  }
  else
  {
    positiveTouchCounter = 0;
  }

  if (positiveTouchCounter >= positiveTouchCounterLimit)
  {
    //ShelfLightState();
    playBeep();
    AUTO_MODE = !AUTO_MODE;
    if (ONLINE)
      Blynk.virtualWrite(vPIN_AUTO_MODE, AUTO_MODE);
    positiveTouchCounter = 0;
    //delay(1000);
  }
}

void playBeep()
{
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
}

void autonomousFill()
{
  if (tank_percent <= 95 && sump_percent > 10 && !PUMP_ON)
  {
    AUTO_MODE = true;
    turnONPump();
  }
}
