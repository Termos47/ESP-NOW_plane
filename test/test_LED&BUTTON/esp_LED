#include <esp_now.h>
#include <WiFi.h>

const int ledPin = 5;
bool ledState = false;

// Универсальная callback-функция, совместимая со всеми версиями
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
#endif
  Serial.println("Signal received!");
  ledState = !ledState;
  digitalWrite(ledPin, ledState);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Receiver MAC: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Регистрация callback с приведением типа
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.println("Receiver ready!");
}

void loop() {}
