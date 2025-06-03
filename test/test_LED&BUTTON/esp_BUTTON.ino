#include <esp_now.h>
#include <WiFi.h>

const int buttonPin = 4;
bool lastButtonState = HIGH;

// Широковещательный адрес
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Sender MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(buttonPin, INPUT_PULLUP);
  Serial.println("Sender ready!");
}

void loop() {
  bool currentButtonState = digitalRead(buttonPin);
  
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    uint8_t data = 1;
    esp_err_t result = esp_now_send(broadcastAddress, &data, sizeof(data));
    
    if (result == ESP_OK) Serial.println("Send success");
    else Serial.println("Send failed");
    
    delay(50);
  }
  
  lastButtonState = currentButtonState;
  delay(10);
}
