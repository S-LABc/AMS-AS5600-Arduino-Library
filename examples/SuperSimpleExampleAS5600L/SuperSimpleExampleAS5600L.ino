/*
 * SuperSimpleExampleAS5600L
 * 
 * Вывода значения угла в градусах от датчика AS5600L в "Монитор порта"
 * 
 * Подключение:
 * AS5600L  Board
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
 * https://ams.com/documents/20143/36005/AS5600L_DS000545_3-00.pdf
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
#include <AMS_AS5600L.h>

// Создаем объект Sensor с указанием ссылки на объект Wire
// По умолчинию адрес датчика на шине I2C 0x40
AS5600L Sensor(&Wire);

void setup() {
  // Настраиваем "Монитор порта"
  Serial.begin(115200);
  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 100кГц
  Sensor.setClock(AS5600_I2C_CLOCK_100KHZ);
}

void loop() {
  Serial.print("Угол в градусах: ");
  // Выводим значения в градусах (от 0 до 360)
  Serial.println(Sensor.getDegreesAngle());
  delay(500);
}
