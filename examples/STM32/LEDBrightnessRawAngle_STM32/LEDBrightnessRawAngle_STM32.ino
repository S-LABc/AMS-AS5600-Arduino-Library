/*
 * LEDBrightnessRawAngle_STM32
 * 
 * Демонстрация управления яркостью светодиода 
 * при помощи датчика AS5600
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA (PB7)
 * SCL   -> SCL (PB6)
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Подключить светодиод между LED_PIN и GND через резистор
 * 3. Загрузить скетч в плату
 * 4. Менять положение магнита
 *
 * Примечания:
 * 1. Скетч использует ядро Arduino_STM32
 * 2. Если используется плата с логическими уровнями 5В, то
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

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Контакт микроконтроллера с подключенным светодиодом
// Подходит только с функцией PWM, например PA0–PA3 PA6–PA10 PB0-PB1 PB6–PB9
#define LED_PIN PA0

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Подключаем ШИМ на контакт со светодиодом
  pinMode(LED_PIN, PWM);

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
  // Приводим диапазон 0-4095 к даипазону 0-65535 (STM32 может 16 бит ШИМ)
  word brightness = map(raw, 0, 4095, 0, 65535);
  
  // Устанавливаем нужную яркость
  pwmWrite(LED_PIN, brightness);
}
