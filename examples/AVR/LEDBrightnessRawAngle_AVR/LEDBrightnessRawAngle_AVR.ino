/*
 * LEDBrightnessRawAngle_AVR
 * 
 * Демонстрация управления яркостью светодиода 
 * при помощи датчика AS5600
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
 * 2. Подключить светодиод между LED_PIN и GND через резистор
 * 3. Загрузить скетч в плату
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

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Контакт микроконтроллера с подключенным светодиодом
// Подходит только с функцией PWM, например ~3, ~5, ~6, ~9, ~10, ~11
#define LED_PIN 9

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Настраиваем контакт со светодиодом на выход
  pinMode(LED_PIN, OUTPUT);

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
  // Получаем значения АЦП для полного круга 0-4095
  word raw = Sensor.getRawAngle();
  // Приводим диапазон 0-4095 к даипазону 0-255 (8 бит ШИМ)
  byte brightness = map(raw, 0, 4095, 0, 255);
  
  // Устанавливаем нужную яркость
  analogWrite(LED_PIN, brightness);
}
