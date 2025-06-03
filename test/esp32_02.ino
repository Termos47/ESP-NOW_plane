#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Настройки MAC-адресов (обратные первой плате)
uint8_t device1Mac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; // MAC первой платы
uint8_t device2Mac[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBD}; // MAC этой платы

// Структура сообщения (должна быть идентичной)
#pragma pack(push, 1)
typedef struct {
  char message[32];
  int counter;
  uint32_t timestamp;
} DataPacket;
#pragma pack(pop)

DataPacket outgoingPacket;
DataPacket incomingPacket;

void printMac(const uint8_t* mac) {
  for(int i = 0; i < 6; i++) {
    Serial.print(mac[i], HEX);
    if(i < 5) Serial.print(":");
  }
}

void OnDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.print("\nSent to: ");
  printMac(mac);
  Serial.printf(" | Status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  memcpy(&incomingPacket, data, sizeof(incomingPacket));
  
  Serial.print("\nReceived from: ");
  printMac(info->src_addr);
  Serial.printf("\nMessage: %s\nCounter: %d\nTimestamp: %lu\n", 
               incomingPacket.message, 
               incomingPacket.counter, 
               incomingPacket.timestamp);
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nDevice 2 Initialization...");

  // Установка MAC
  WiFi.mode(WIFI_STA);
  if(esp_wifi_set_mac(WIFI_IF_STA, device2Mac) != ESP_OK) {
    Serial.println("Failed to set MAC!");
    while(true) delay(1000);
  }

  // Инициализация ESP-NOW
  if(esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while(true) delay(1000);
  }

  // Регистрация колбэков
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Настройка пира
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, device1Mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    while(true) delay(1000);
  }

  Serial.print("Device MAC: ");
  printMac(device2Mac);
  Serial.println("\nReady for communication...");
}

void loop() {
  static uint32_t lastSend = 0;
  if(millis() - lastSend >= 3000) { // Отправка с интервалом 3 секунды
    lastSend = millis();

    // Формирование пакета
    snprintf(outgoingPacket.message, sizeof(outgoingPacket.message), 
            "Greetings from Device 2");
    outgoingPacket.counter++;
    outgoingPacket.timestamp = millis();

    // Отправка данных
    esp_err_t result = esp_now_send(device1Mac, 
                                  (uint8_t*)&outgoingPacket, 
                                  sizeof(outgoingPacket));

    if(result != ESP_OK) {
      Serial.println("Error sending data!");
    }
  }
}
