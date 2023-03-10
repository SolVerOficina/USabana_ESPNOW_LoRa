
//MAC ADRESS DE ANTENA ROJA: 30:C6:F7:0D:D2:28


#include <analogWrite.h>
#include <SPI.h>
#include <LoRa.h>
//#include <Arduino_JSON.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define PIN_RED    25 // 
#define PIN_GREEN  26 // 
#define PIN_BLUE   27 //

//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

bool banderaWifi;

int rest;  
int reset_time = 1000;
int sensor_carro = 0;
float sv_vard;  

typedef struct struct_message {
      String senal;
} struct_message;

// Crear struct_message llamado myData
struct_message myData;

// Insert your SSID
constexpr char WIFI_SSID[] = "Soluciones Verticales SAS";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

String smsj;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  analogWrite(PIN_RED,   0);
    analogWrite(PIN_GREEN, 255);
    analogWrite(PIN_BLUE,  0);
    delay(1000);
  analogWrite(PIN_RED,   0);
    analogWrite(PIN_GREEN, 255);
    analogWrite(PIN_BLUE,  0);
    delay(1000);
  // Copies the sender mac address to a string
  char macStr[18];     
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
smsj = myData.senal;

 //imprime los valores en la consola-------
//    Serial.print("Clave: ");
//  Serial.println(myData.clave);
  Serial.print("Mensaje: ");
  Serial.println(myData.senal);
  //banderaWifi = true;
  delay(1500);
  send_lora_paquet();
}

void succes(){
  Serial.println("succes");
   analogWrite(PIN_RED,   255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  255);
  delay(1500);
   analogWrite(PIN_RED,   255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  255);
  delay(1500);
   analogWrite(PIN_RED,   255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  255);
  delay(1500);
   analogWrite(PIN_RED,   255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  255);
    delay(1000);
   analogWrite(PIN_RED,   0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  0);
      delay(1000);
   analogWrite(PIN_RED,   255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  255);
      delay(1000);
   analogWrite(PIN_RED,   0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  0);
}

int counter = 0;

void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(PIN_RED,   OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE,  OUTPUT); 
   Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

 int32_t channel = getWiFiChannel(WIFI_SSID);
  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial);
  
 // Init ESP-NOW
  if (esp_now_init() != ESP_OK){
    Serial.println("Error initializing ESP-NOW");
    return;
  }else{
    Serial.println("ESP init okay!");
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(915E6)) {
    Serial.println(".");
    delay(500);
  }
//    // Setup Spreading Factor (6 ~ 12)
//  LoRa.setSpreadingFactor(10);
//  
//  // Setup BandWidth, option: 7800,10400,15600,20800,31250,41700,62500,125000,250000,500000
//  //Lower BandWidth for longer distance.
//  LoRa.setSignalBandwidth(125000);
//  
//  // Setup Coding Rate:5(4/5),6(4/6),7(4/7),8(4/8) 
//  LoRa.setCodingRate4(5);
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  //addreess oficina bogota ---------------- 0x18
  LoRa.setSyncWord(0x18);

  Serial.println("LoRa Initializing OK!");
  delay(500);
  //send_lora_paquet();
}

void loop() {
  if(rest > reset_time){
    ESP.restart();
      delay(1000);
 //send_lora_paquet();
    rest = 0;
  }else{
    rest++;
  //  Serial.println("StandBy");
    analogWrite(PIN_RED,   255);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  0);
  }
  delay(1000);
}

void send_lora_paquet(){
    Serial.print("Sending packet: ");
  LoRa.beginPacket();
  LoRa.print(smsj);
  LoRa.endPacket();
  Serial.println("DATA SENT BY LORA");
succes();
}
