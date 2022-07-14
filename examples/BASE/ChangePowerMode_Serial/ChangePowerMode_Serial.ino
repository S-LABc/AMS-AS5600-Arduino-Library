/*
 * ChangePowerMode_Serial
 *  
 * Демонстрация изменения режима питания датчика AS5600
 * путем отправление команд через "Монитор порта"
 * 
 * Примечания:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
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
 * 4. Отправлять 0, 1, 2, 3
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
  // Проверка данных в буфере UART
  if(Serial.available()) {
    uint8_t num_mode = Serial.read(); // Чтение команды из "Монитора порта"

    // Установка разных режимов
    if(num_mode == '0') {
      if(Sensor.enableNomPowerModeVerify()) { // Включить нормальный режим питания с проверкой
        Serial.println("Режим: Нормальный");
      }else {
        Serial.println("Ошибка установки Нормального режима");
      }
    }else if(num_mode == '1') {
      Sensor.enableLowPowerMode1(); // Включить пониженный режим питания 1
      Serial.println("Режим: Пониженный №1");
    }else if(num_mode == '2') {
      Sensor.enableLowPowerMode2(); // Включить пониженный режим питания 2
      Serial.println("Режим: Пониженный №2");
    }else if(num_mode == '3') {
      Sensor.enableLowPowerMode3(); // Включить пониженный режим питания 3
      Serial.println("Режим: Пониженный №3");
    }else {
      Serial.println("Ошибка! Можно вводить только: 0, 1, 2, 3"); // Если отправленное значение режима не 0, 1, 2, 3
    }
  }
}

/*
 * Все методы управления питанием
 * 
 * AS5600PowerModes getPowerMode(); // Получить текущий режим питания
 * void setPowerMode(AS5600PowerModes _power_mode); // Установить новый режим питания
 * bool setPowerModeVerify(AS5600PowerModes _power_mode); // с подтверждением
 * 
 * _power_mode:
 * AS5600_NOM_POWER_MODE
 * AS5600_LOW_POWER_MODE_1
 * AS5600_LOW_POWER_MODE_2
 * AS5600_LOW_POWER_MODE_3
 * 
 * void enableNomPowerMode(); // Включить нормальный режим питания
 * bool enableNomPowerModeVerify(); // с подтверждением
 * 
 * void enableLowPowerMode1(); // Включить пониженный режим питания 1
 * bool enableLowPowerMode1Verify(); // с подтверждением
 * 
 * void enableLowPowerMode2(); // Включить пониженный режим питания 2
 * bool enableLowPowerMode2Verify(); // с подтверждением
 * 
 * void enableLowPowerMode3(); // Включить пониженный режим питания 3
 * bool enableLowPowerMode3Verify(); // с подтверждением
 */
