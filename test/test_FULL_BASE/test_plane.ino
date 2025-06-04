#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <EEPROM.h>

// Конфигурация пинов
const int MOTOR_PIN = 13;
const int SERVO1_PIN = 12;
const int SERVO2_PIN = 14;
const int SERVO3_PIN = 27;
const int LED_PIN = 2;

// Настройки ESP-NOW
const uint8_t ENCRYPT_KEY[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                              0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};

// Параметры управления
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;
const uint32_t CONNECTION_TIMEOUT = 500;

// Структура данных управления
#pragma pack(push, 1)
typedef struct ControlData {
  uint16_t joy1Y;
  uint16_t joy1X;
  uint16_t joy2Y;
  uint16_t joy2X;
  uint32_t packet_id;
  uint8_t checksum;
} ControlData;
#pragma pack(pop)

// Структура калибровки сервоприводов
typedef struct ServoCalibration {
  int16_t servo1_min = 1000;
  int16_t servo1_max = 2000;
  int16_t servo1_center = 1500;
  int16_t servo2_min = 1000;
  int16_t servo2_max = 2000;
  int16_t servo2_center = 1500;
  int16_t servo3_min = 1000;
  int16_t servo3_max = 2000;
  int16_t servo3_center = 1500;
} ServoCalibration;

// Глобальные переменные
ControlData receivedData;
uint32_t last_received = 0;
ServoCalibration servoCalib;

// Фильтры
const int FILTER_SIZE = 5;
uint16_t filterBuffer[4][FILTER_SIZE] = {0};
uint8_t filterIndex = 0;

// Объекты сервоприводов
Servo motor;
Servo servo1;
Servo servo2;
Servo servo3;

// Прототипы функций
void loadServoCalibration();
void activateFailSafe();
void updateFilter(uint8_t channel, uint16_t value);
uint16_t applyFilter(uint8_t channel);
int mapServoValue(uint16_t input, int min_val, int max_val, int center);
uint8_t calculateChecksum(void* data, size_t len);
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len); // Объявление функции обратного вызова

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // Загрузка калибровки из EEPROM
  EEPROM.begin(sizeof(ServoCalibration));
  loadServoCalibration();
  
  // Инициализация сервоприводов
  if (!motor.attach(MOTOR_PIN)) Serial.println("Failed to attach motor");
  if (!servo1.attach(SERVO1_PIN)) Serial.println("Failed to attach servo1");
  if (!servo2.attach(SERVO2_PIN)) Serial.println("Failed to attach servo2");
  if (!servo3.attach(SERVO3_PIN)) Serial.println("Failed to attach servo3");
  
  // Безопасная инициализация
  motor.writeMicroseconds(MOTOR_MIN);
  servo1.writeMicroseconds(servoCalib.servo1_center);
  servo2.writeMicroseconds(servoCalib.servo2_center);
  servo3.writeMicroseconds(servoCalib.servo3_center);
  
  delay(3000);  // Ожидание инициализации ESC

  // Инициализация WiFi и ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while(1) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }

  // Установка ключа шифрования
  esp_now_set_pmk((uint8_t *)ENCRYPT_KEY);

  // Регистрация callback-функции приема данных
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("Receiver initialized and ready");
}

// Функция обратного вызова для приема данных ESP-NOW
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(ControlData)) return;
  
  ControlData packet;
  memcpy(&packet, data, len);
  
  // Проверка контрольной суммы
  uint8_t checksum = packet.checksum;
  packet.checksum = 0;
  if (calculateChecksum(&packet, len) != checksum) {
    Serial.println("Checksum error!");
    return;
  }
  
  memcpy(&receivedData, &packet, len);
  last_received = millis();
  digitalWrite(LED_PIN, HIGH);
  
  // Ответное подтверждение (опционально)
  esp_now_send(mac, (uint8_t*)"ACK", 3);
}

void loop() {
  static int currentThrottle = MOTOR_MIN;
  static uint32_t last_packet_id = 0;
  
  // Проверка таймаута соединения
  if (millis() - last_received > CONNECTION_TIMEOUT) {
    activateFailSafe();
    digitalWrite(LED_PIN, LOW);
    return;
  }
  
  // Пропуск дубликатов пакетов
  if (receivedData.packet_id == last_packet_id) return;
  last_packet_id = receivedData.packet_id;
  
  // Фильтрация значений
  updateFilter(0, receivedData.joy1Y);
  updateFilter(1, receivedData.joy1X);
  updateFilter(2, receivedData.joy2Y);
  updateFilter(3, receivedData.joy2X);
  
  // Плавное управление мотором
  int targetThrottle = map(applyFilter(0), 0, 4095, MOTOR_MIN, MOTOR_MAX);
  if (abs(targetThrottle - currentThrottle) > 10) {
    currentThrottle += (targetThrottle > currentThrottle) ? 1 : -1;
  }
  motor.writeMicroseconds(constrain(currentThrottle, MOTOR_MIN, MOTOR_MAX));
  
  // Управление сервоприводами с калибровкой
  servo1.writeMicroseconds(mapServoValue(applyFilter(1), 
                          servoCalib.servo1_min, 
                          servoCalib.servo1_max, 
                          servoCalib.servo1_center));
  
  servo2.writeMicroseconds(mapServoValue(applyFilter(2), 
                          servoCalib.servo2_min, 
                          servoCalib.servo2_max, 
                          servoCalib.servo2_center));
  
  servo3.writeMicroseconds(mapServoValue(applyFilter(3), 
                          servoCalib.servo3_min, 
                          servoCalib.servo3_max, 
                          servoCalib.servo3_center));
  
  // Отладочный вывод (раз в 100 мс)
  static uint32_t last_print = 0;
  if (millis() - last_print > 100) {
    Serial.printf("Throttle: %d, Servo1: %d, Servo2: %d, Servo3: %d\n", 
                 currentThrottle,
                 mapServoValue(applyFilter(1), servoCalib.servo1_min, servoCalib.servo1_max, servoCalib.servo1_center),
                 mapServoValue(applyFilter(2), servoCalib.servo2_min, servoCalib.servo2_max, servoCalib.servo2_center),
                 mapServoValue(applyFilter(3), servoCalib.servo3_min, servoCalib.servo3_max, servoCalib.servo3_center));
    last_print = millis();
  }
  
  delay(10);
}

// Функции ---------------------------------------------------------------------

void loadServoCalibration() {
  if (EEPROM.read(0) != 0xFF) { // Проверка, была ли записана калибровка
    EEPROM.get(0, servoCalib);
  } else {
    // Значения по умолчанию
    servoCalib.servo1_min = 1000;
    servoCalib.servo1_max = 2000;
    servoCalib.servo1_center = 1500;
    servoCalib.servo2_min = 1000;
    servoCalib.servo2_max = 2000;
    servoCalib.servo2_center = 1500;
    servoCalib.servo3_min = 1000;
    servoCalib.servo3_max = 2000;
    servoCalib.servo3_center = 1500;
  }
}

void activateFailSafe() {
  motor.writeMicroseconds(MOTOR_MIN);
  servo1.writeMicroseconds(servoCalib.servo1_center);
  servo2.writeMicroseconds(servoCalib.servo2_center);
  servo3.writeMicroseconds(servoCalib.servo3_center);
}

void updateFilter(uint8_t channel, uint16_t value) {
  filterBuffer[channel][filterIndex] = value;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
}

uint16_t applyFilter(uint8_t channel) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < FILTER_SIZE; i++) {
    sum += filterBuffer[channel][i];
  }
  return sum / FILTER_SIZE;
}

int mapServoValue(uint16_t input, int min_val, int max_val, int center) {
  const int deadzone = 50; // Мертвая зона вокруг центра
  if (abs(input - 2048) < deadzone) return center;
  if (input < 2048) return map(input, 0, 2048-deadzone, min_val, center);
  else return map(input, 2048+deadzone, 4095, center, max_val);
}

uint8_t calculateChecksum(void* data, size_t len) {
  uint8_t* bytes = (uint8_t*)data;
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum ^= bytes[i]; // Простая XOR контрольная сумма
  }
  return sum;
}
