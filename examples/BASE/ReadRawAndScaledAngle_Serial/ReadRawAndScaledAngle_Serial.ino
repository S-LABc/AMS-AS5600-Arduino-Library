/*
 * ReadRawAndScaledAngle_Serial
 * 
 * Демонстрация вывода значений угла от датчика AS5600 в "Монитор порта"
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

  // Пока не подключен датчик
  while (!Sensor.isConnected()) {
    // Выводим сообщение об отсутствии датчика
    Serial.println("AS5600 не обнаружен!");
    Serial.println("Проверьте I2C шину");
    delay(1000);
  }
  // Выводим сообщение о наличии датчика
  Serial.println("AS5600 обнаружен!");

  // Пока датчик не обнаружил магнит
  while (!Sensor.isMagnetDetected()) {
    // Выводим сообщение об отсутствии магнита
    Serial.println("Магнит не обнаружен!");
    Serial.println("Возможно он слабый");
    delay(1000);
  }
  // Выводим сообщение о наличии магнита
  Serial.println("Магнит обнаружен!");

  // Задаем сектор в котором будет двигаться магнит (кусок полного круга)
  Sensor.setZeroPosition(0); // От 0
  Sensor.setMaxPosition(2048); // До 180 градусов
}

void loop() {
  // Значения АЦП для полного круга (от 0 до 4095)
  Serial.print("Угол АЦП: ");
  Serial.println(Sensor.getRawAngle());

  // Значения АЦП для сектора (от 0 до 4095)
  Serial.print("Угол сектора: ");
  Serial.println(Sensor.getScaledAngle());

  // Разделение и задержка
  Serial.println();
  delay(100);
}
