#include <esp_now.h>
#include <WiFi.h>

const int JOYSTICK_X_PIN = 34;  // Пин оси X джойстика

// Фильтрация значений
const int SAMPLE_SIZE = 15;
int samples[SAMPLE_SIZE];
int sampleIndex = 0;

// Широковещательный адрес
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Структура для передачи данных
typedef struct message {
  uint16_t joystickValue;  // Сырое значение 0-4095
} message;

message dataToSend;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  // Инициализация ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Регистрация пира
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Инициализация буфера сглаживания
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    samples[i] = analogRead(JOYSTICK_X_PIN);
  }
  
  Serial.println("Joystick Transmitter Ready");
}

void loop() {
  // Считывание и фильтрация значений джойстика
  samples[sampleIndex] = analogRead(JOYSTICK_X_PIN);
  sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
  
  long sum = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sum += samples[i];
  }
  int smoothedValue = sum / SAMPLE_SIZE;

  // Упаковка данных для отправки
  dataToSend.joystickValue = smoothedValue;

  // Отправка данных
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));
  
  if (result == ESP_OK) {
    Serial.print("Sent value: ");
    Serial.println(smoothedValue);
  } else {
    Serial.println("Error sending data");
  }
  
  delay(20); // Интервал отправки
}
