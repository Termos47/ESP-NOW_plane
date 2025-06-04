#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

// Конфигурация пинов
const int JOYSTICK1_Y_PIN = 32;
const int JOYSTICK1_X_PIN = 33;
const int JOYSTICK2_Y_PIN = 34;
const int JOYSTICK2_X_PIN = 35;
const int LED_PIN = 2;
const int CALIB_BUTTON = 15;

// Настройки ESP-NOW
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t ENCRYPT_KEY[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 
                              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};

// Структура данных
#pragma pack(push, 1)
typedef struct ControlData {
  uint16_t joy1Y;
  uint16_t joy1X;
  uint16_t joy2Y;
  uint16_t joy2X;
  uint32_t packet_id = 0;
  uint8_t checksum = 0;
} ControlData;
#pragma pack(pop)

ControlData controlData;
bool peer_connected = false;
uint32_t last_ack_time = 0;
const uint32_t CONNECTION_TIMEOUT = 500;

// Калибровка
typedef struct CalibrationData {
  uint16_t joy1Y_min = 0;
  uint16_t joy1Y_max = 4095;
  // ... другие калибровочные значения
} CalibrationData;

CalibrationData calib;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CALIB_BUTTON, INPUT_PULLUP);
  WiFi.mode(WIFI_STA);
  
  // Инициализация EEPROM
  EEPROM.begin(sizeof(CalibrationData));
  loadCalibration();

  // Инициализация ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while(1) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Настройка шифрования
  esp_now_set_pmk((uint8_t *)ENCRYPT_KEY);

  // Регистрация пира
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = true;
  memcpy(peerInfo.lmk, ENCRYPT_KEY, 16);
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Регистрация callback
  esp_now_register_send_cb([](const uint8_t *mac, esp_now_send_status_t status) {
    peer_connected = (status == ESP_NOW_SEND_SUCCESS);
    last_ack_time = millis();
  });

  Serial.println("Transmitter Ready");
}

void loop() {
  static uint32_t last_send = 0;
  
  // Калибровка по кнопке
  if(digitalRead(CALIB_BUTTON) == LOW) {
    calibrateJoysticks();
  }

  // Чтение и калибровка значений
  controlData.joy1Y = calibrateValue(analogRead(JOYSTICK1_Y_PIN), calib.joy1Y_min, calib.joy1Y_max);
  // ... остальные оси

  // Управление светодиодом связи
  digitalWrite(LED_PIN, peer_connected);
  
  // Проверка таймаута соединения
  if(millis() - last_ack_time > CONNECTION_TIMEOUT) {
    peer_connected = false;
    Serial.println("Connection lost!");
  }

  // Отправка данных (20 мс интервал)
  if(millis() - last_send >= 20) {
    last_send = millis();
    
    // Подготовка пакета
    controlData.packet_id++;
    controlData.checksum = calculateChecksum(&controlData, sizeof(controlData)-1);
    
    esp_err_t result = esp_now_send(broadcastAddress, 
                                  (uint8_t *)&controlData, 
                                  sizeof(controlData));
    
    if(result != ESP_OK) {
      Serial.println("Send error");
    }
  }
  
  delay(1);
}

// Функции калибровки и безопасности
void calibrateJoysticks() {
  Serial.println("Starting calibration...");
  // Реализация калибровки
  // ...
  saveCalibration();
}

uint16_t calibrateValue(uint16_t raw, uint16_t min_val, uint16_t max_val) {
  return constrain(map(raw, min_val, max_val, 0, 4095), 0, 4095);
}

uint8_t calculateChecksum(void* data, size_t len) {
  uint8_t* bytes = (uint8_t*)data;
  uint8_t sum = 0;
  for(size_t i=0; i<len; i++) sum ^= bytes[i];
  return sum;
}

void loadCalibration() {
  EEPROM.get(0, calib);
}

void saveCalibration() {
  EEPROM.put(0, calib);
  EEPROM.commit();
}
