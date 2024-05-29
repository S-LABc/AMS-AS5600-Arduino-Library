/*
 * MagnetState_Serial
 * 
 * Пололучение информации о состоянии
 * магнита от датчика AS5600
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Примеяания:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта"
 * 4. Приближать и удалять магнит
 * 5. Наблюдать результат
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
  if(Sensor.isMagnetDetected()) {
    Serial.print("Магнит обнаружен, ");
    if(Sensor.isMagnetTooWeak()) {
      Serial.println("но он слишком слабый!");
    } else if(Sensor.isMagnetTooStrong()) {
      Serial.println("но он слишком сильный!");
    } else {
      Serial.println("и он что надо!");
    }
  } else {
    Serial.println("Магнит не обнаружен!");
  }
  
  Serial.println();
  delay(1000);
}
