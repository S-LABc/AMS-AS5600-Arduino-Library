/*
 * LEDBrightnessRawAngle_ESP32
 * 
 * Демонстрация управления яркостью светодиода 
 * на микроконтроллере ESP32 при помощи датчика AS5600
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA (GPIO21)
 * SCL   -> SCL (GPIO22)
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Подключить светодиод между LED_PIN и GND через резистор
 * 3. Загрузить скетч в плату
 * 4. Менять положение магнита
 *
 * Примечание:
 * Скетч использует ядро arduino-esp32
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 * 
 * Зависимости:
 * https://github.com/espressif/arduino-esp32
 *
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

// Контакт микроконтроллера с подключенным светодиодом
// Подходит любой который можно настроить на цифровой выход
#define LED_PIN 19

// Параметры ШИМ
#define LED_CHANNEL    0 // Канал ШИМ 0-15
#define LED_FREQENCY   10000 // Несущая частота ШИМ в Гц
#define LED_RESOLUTION 12 // Разрешение ШИМ 1-16 бит

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Настравиаем параметры ШИМ
  ledcSetup(LED_CHANNEL, LED_FREQENCY, LED_RESOLUTION);
  // Подключаем ШИМ на контакт со светодиодом
  ledcAttachPin(LED_PIN, LED_CHANNEL);

  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
}

void loop() {
  // Получаем значения АЦП для полного круга 0-4095
  word brightness = Sensor.getRawAngle();
  
  // Разрешение датчика 12, разрешение ШИМ установлено 12 бит
  ledcWrite(LED_CHANNEL, brightness); // В данном случаи brightness это скважность ШИМ сигнала и реальный угол
}
