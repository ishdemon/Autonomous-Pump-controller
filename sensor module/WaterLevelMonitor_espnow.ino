#include <SimpleTimer.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#define ECHOPIN                 D6 // Echo Pin D5
#define TRIGPIN                 D5 // Trigger Pin D6
//MAC address
uint8_t broadcastAddress[] = {""};  //Put reciever's MAC address

typedef struct struct_message {
  long d;
  long stamp;
} struct_message;

// Create a struct_message called myData
struct_message myData;
long duration, distance;
SimpleTimer timer;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

void setup()
{
  // Debug console
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
  WiFi.mode(WIFI_STA);
  WiFi.begin("", "", 4, NULL, false);
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  if (esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 4, NULL, 0) != 0){
    Serial.println("Failed to add peer");
    return;
  }
  timer.setInterval(2000L, getDepth);

}

void loop()
{
  timer.run();
}

void getDepth() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIGPIN, LOW);

  duration = pulseIn(ECHOPIN, HIGH);
  distance = duration / 58.2;
  Serial.print(distance);
  Serial.println("cm");
  myData.d = distance;
  myData.stamp = millis();
  esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
}
