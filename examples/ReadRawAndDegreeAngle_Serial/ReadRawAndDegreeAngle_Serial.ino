/*
 * ReadRawAndDegreeAngle_Serial
 * 
 * Демонтрация вывода значений угла в градусах от датчика AS5600 в "Монитор порта"
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

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  Serial.begin(115200);

  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();

  // Пока не подключен датчик
  while (!Sensor.isConnected()) {
    // Выводим сообщение об отсутствии датчика
    Serial.println("AS5600 not detected!");
    delay(1000);
  }
  // Выводим сообщение о наличии датчика
  Serial.println("AS5600 detected!");

  // Пока датчик не обнаружил магнит
  while (!Sensor.isMagnetDetected()) {
    // Выводим сообщение об отсутствии магнита
    Serial.println("Magnet not detected!");
    delay(1000);
  }
  // Выводим сообщение о наличии магнита
  Serial.println("Magnet detected!");
}

void loop() {
  // Получаем значения АЦП для полного круга
  uint16_t raw = Sensor.getRawAngle();
  
  // Выводим "сырые" значения (от 0 до 4095)
  Serial.print("Raw Angle: ");
  Serial.println(raw);

  // Выводим значения в градусах (от 0 до 360)
  Serial.print("Degree Angle: ");
  Serial.println(raw * 0.08791); // 360/4095=0.0879120879120879, 5 знаков после точки для АЦП 12 бит достаточно

  // Разделение и задержка
  Serial.println();
  delay(100);
}
