#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

const int ESC_PIN = 13;         // Пин управления ESC
Servo esc;

// Параметры управления
const int NEUTRAL = 1500;       // Нейтральное положение (стоп)
const int MAX_FORWARD = 2000;   // Максимальный импульс вперед
const int MAX_REVERSE = 1000;   // Максимальный импульс назад
const uint16_t CENTER = 2756;   // Центральное значение (стоп)
const uint16_t DEADZONE = 10;  // Зона нечувствительности

// Структура для приема данных
typedef struct message {
  uint16_t joystickValue;
} message;

message receivedData;
bool newDataAvailable = false;

// Callback-функция для приема данных
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
#else
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
#endif
  if (data_len == sizeof(receivedData)) {
    memcpy(&receivedData, data, data_len);
    newDataAvailable = true;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("Motor Receiver Ready");

  // Инициализация ESC
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(NEUTRAL);
  delay(5000);  // Ожидание инициализации ESC

  // Инициализация ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
}

void loop() {
  if (newDataAvailable) {
    // Обработка нового значения джойстика
    uint16_t joystickValue = receivedData.joystickValue;
    
    // Простое управление без плавности
    if (abs(joystickValue - CENTER) <= DEADZONE) {
      // В пределах мертвой зоны - стоп
      esc.writeMicroseconds(NEUTRAL);
      Serial.print(joystickValue);
      Serial.println(" -> STOP");
    } 
    else if (joystickValue > CENTER) {
      // Выше центра - полный вперед
      esc.writeMicroseconds(MAX_FORWARD);
      Serial.print(joystickValue);
      Serial.println(" -> FULL FORWARD");
    }
    else {
      // Ниже центра - полный назад
      esc.writeMicroseconds(MAX_REVERSE);
      Serial.print(joystickValue);
      Serial.println(" -> FULL REVERSE");
    }
    
    newDataAvailable = false;
  }
  delay(20);
}
