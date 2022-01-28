/*
 * ReadAngle_VisuallyPC
 * 
 * Демонтрация вывода данных от датчика AS5600 на компьютер
 * для отображения в программе "AS5600 Visually.exe"
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Примечения:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 *
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть программу [библиотеки]\AMS-AS5600-Arduino-Library\addons\AS5600 Visually\AS5600 Visually.exe
 * 4. Выбрать com port и нажать Connect
 * 5. Менять положение магнита
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 * 
 * Контакты:
 ** YouTube - https://www.youtube.com/channel/UCbkE52YKRphgkvQtdwzQbZQ
 ** Telegram - https://www.t.me/slabyt
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Настраиваем скорость ТОЛЬКО 115200
  Serial.begin(115200);

  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
}

void loop() {
  // Отправляем данные на компьтер, где отображаем их в "AS5600 Visually.exe"
  Serial.println(Sensor.getRawAngle());
  delay(1);
}
