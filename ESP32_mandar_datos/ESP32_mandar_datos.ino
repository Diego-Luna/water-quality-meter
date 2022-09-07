// internet
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "INFINITUM4F18_2.4";
const char* password = "EsnrA23be5";

String serverName = "http://acercamientos-al-agua.uc.r.appspot.com/create-sensor/";

unsigned long myTime;
unsigned long myTime_old = 0;
bool led_status = false;


// lora
#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define ss 16
#define rst 5
#define dio0 26

bool Error_wifi = false;

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  //initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(915E6)) {
    // Serial.println(".");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  LoRa.setSpreadingFactor(12);
  // Serial.println("LoRa Initializing OK!");

  //  Wifi
  WiFi.begin(ssid, password);
  // Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    // Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();

  myTime = millis();
  if (Error_wifi == false && myTime >= (500 + myTime_old)) {
    digitalWrite(LED_BUILTIN, (led_status == true) ? HIGH : LOW);
    myTime_old = myTime;
    led_status = !led_status;
  } else if (myTime < myTime_old) {
    myTime_old = myTime;
  }

  if (packetSize) {
    // received a packet
    // Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.println(LoRaData);
      send_to_internet(serverName + LoRaData);
    }

    // print RSSI of packet
    // Serial.print("' with RSSI ");
    // Serial.println(LoRa.packetRssi());
  }
}

void send_to_internet(String lora_string)
{
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;


    // Serial.println();
    // Serial.print(lora_string);
    // Serial.println();

    http.begin(lora_string.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      // Serial.print("HTTP Response code: ");
      // Serial.println(httpResponseCode);
      String payload = http.getString();
      Serial.println(payload);
    }
    else {
      // Serial.print("Error code: ");
      // Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
    digitalWrite(LED_BUILTIN, LOW);
  }
  else {
    // Serial.println("WiFi Disconnected");
    Error_wifi = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
