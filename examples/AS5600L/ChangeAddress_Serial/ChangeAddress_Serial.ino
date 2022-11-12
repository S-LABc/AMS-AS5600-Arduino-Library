/*
 * ChangeAddress_Serial
 * 
 * Управение адресом на шине I2C датчика AS5600L через "Монитор порта"
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
 * 4. Менять положение магнита и отправлять символ "c"
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

byte cmd = 0; // Хранит порядковый номер установки адреса датчика

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600L Sensor(&Wire); // По умолчинию адрес датчика на шине I2C 0x40
// Можно указать адрес сразу в конструкторе, если он известен, например 0x51
//AS5600L Sensor(&Wire, 0x51);

void setup() {
  // Настраиваем "Монитор порта"
  Serial.begin(115200);
  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 100кГц
  // На частоте выше 300кГц не всегда работает
  Sensor.setClock(AS5600_I2C_CLOCK_100KHZ);
}

void loop() {
  if (Serial.available()) {
    if(Serial.read() == 'c') {
      if (cmd >= 3) { // Ограничение максимума вариантов
        cmd = 0;
      }
      switch (cmd) { // Меняем адрес
        case 0:
          Sensor.setAddressI2C(0x51);
          break;
        case 1:
          Sensor.setAddressI2C(0x42);
          break;
        case 2:
          Sensor.setAddressI2C(0x36);
          break;
      }
      cmd++;
    }
  } else {
    Serial.print("Адрес: ");
    Serial.print("0x");
    byte addr = Sensor.getAddressI2C(); // Получаем адрес датчика
    if (addr < 16) { // Для удобства представления. Например, вместо 7, будет 0x07
      Serial.print("0");
    }
    Serial.print(addr, HEX); // Выводим адрес в шестнадцатеричном виде
    Serial.print(", Угол: ");
    Serial.println(Sensor.getDegreesAngle()); // Выводим угол в градусах
    Serial.println();
  }

  delay(300);
}
