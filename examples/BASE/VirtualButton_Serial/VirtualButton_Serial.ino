/*
 * VirtualButton_Serial
 * 
 * Пример использования программной кнопки у датчика AS5600
 * 
 * Подключение датчика:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 *
 * Примечания:
 * 1. Кнопка реализованная исключительно программным способом
 * 2. Для определения состояния кнопки используются значения
 *    из регистра AGC (метод библиотеки getAutomaticGainControl())
 * 3. Для корректной работы необходимо выполнить калибровку
 *    при помощи CALIBRATION_BUTTON
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта" на скорости 115200
 * 4. Приближать/удалять магнит
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 * 
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** YouTube - https://www.youtube.com/channel/UCbkE52YKRphgkvQtdwzQbZQ
 ** Telegram - https://www.t.me/slabyt
 ** Канал в Telegram - https://www.t.me/t_slab
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

// Раскоментировать для калибровки значений MIN_AGC и MAX_AGC
//#define CALIBRATION_BUTTON

// Значения AGC при перемещении магнита вдоль оси вращения
#define MIN_AGC 99  // Минимальное значение возвращаемое методом getAutomaticGainControl()
#define MAX_AGC 115 // Максимальное значение возвращаемое методом getAutomaticGainControl()
#define DELTA_DIV 5 // Допустимое отклонение минимального и максимального значений

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() { 
  // Настройка и запуск ком порта
  Serial.begin(115200);

  // Настройка границ срабатывания кнопки и отклонения
  Sensor.setButtonMinAGC(MIN_AGC); // getButtonMinAGC()
  Sensor.setButtonMaxAGC(MAX_AGC); // getButtonMaxAGC()
  Sensor.setButtonDeviation(DELTA_DIV); // getButtonDeviation()
  
  // Запускаем соединение с датчиком
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
}

void loop() {
#ifdef CALIBRATION_BUTTON
  handleCalibration();
#else
  handleButton();
#endif
}

void handleButton() {
  // Если виртуальная кнопка датчика НАЖАТА
  if(Sensor.isButtonPressed()) {
    Serial.println("Кнопка НАЖАТА");
  }

  // Если виртуальная кнопка датчика ОТПУЩЕНА
  if(Sensor.isButtonReleased()) {
    Serial.println("Кнопка ОТПУЩЕНА");
  }
}

void handleCalibration() {
  Serial.print("AGC ");
  Serial.println(Sensor.getAutomaticGainControl());
}
