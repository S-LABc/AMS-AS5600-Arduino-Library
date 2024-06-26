/*
 * ReadMagnitudeAndAGC_Serial
 * 
 * Демонстрация вывода значений магнитуды и автоусиления
 * от датчика AS5600 в "Монитор порта"
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
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта"
 * 4. Приближать и отдалять магнит
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
 *
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2024. v1.3 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  Serial.begin(115200);

  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Sensor.setClock(AS5600_I2C_CLOCK_100KHZ); // 100кГц
  //Sensor.setClock(AS5600_I2C_CLOCK_1MHZ); // 1МГц
  //Sensor.setClock(725000); // Пользовательское значение 725кГц
}

void loop() {
  Serial.print("Магнитуда: ");
  Serial.println(Sensor.getMagnitude()); // Значение магнитуды
  Serial.print("Автоусиление: ");
  Serial.println(Sensor.getAutomaticGainControl()); // Значение автоусиления AGC. При VCC = 5В -> 0 - 255, при VCC = 3.3В -> 0 - 128

  /*
  // Или через ссылку
  word magnitude = 0;
  byte agc = 0;
  Sensor.getMagnitude(magnitude);
  Sensor.getAutomaticGainControl(agc);
  Serial.print("Магнитуда: ");
  Serial.println(magnitude);
  Serial.print("Автоусиление: ");
  Serial.println(agc);
  */
  
  Serial.println();
  delay(50);
}
