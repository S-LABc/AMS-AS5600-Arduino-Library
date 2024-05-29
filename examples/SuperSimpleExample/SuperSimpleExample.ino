/*
 * SuperSimpleExample
 * 
 * Вывода значения угла в градусах от датчика AS5600 в "Монитор порта"
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта"
 * 4. Менять положение магнита
 *
 * Примечания:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
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
 * Copyright (C) 2024. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Настраиваем "Монитор порта"
  Serial.begin(115200);
  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
}

void loop() {
  Serial.print("Угол в градусах: ");
  // Выводим значения в градусах (от 0 до 360)
  Serial.println(Sensor.getDegreesAngle());

  /*
  // Или через ссылку
  float degrees_ang = 0;
  Sensor.getDegreesAngle(degrees_ang);
  Serial.print("Угол в градусах: ");
  Serial.println(degrees_ang);
  */

  delay(100);
}
