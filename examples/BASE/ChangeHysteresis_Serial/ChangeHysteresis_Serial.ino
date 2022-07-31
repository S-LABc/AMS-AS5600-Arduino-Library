/*
 * ChangeHysteresis_Serial
 *  
 * Демонстрация изменения гистерезиса датчика AS5600
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
 * 4. Отправлять 0, 1, 2 или 3
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 *
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
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
  // Проверка данных в буфере UART
  if(Serial.available()) {
    uint8_t num_mode = Serial.read(); // Чтение команды из "Монитора порта"

    // Установка разных режимов
    if(num_mode == '0') {
      Sensor.disableHysteresis(); // Выключить гистерезис
      Serial.println("Гистерезис выключен");
    }else if(num_mode == '1') {
      Sensor.enableHysteresis1LSB(); // Включить на 1 LSB
      Serial.println("Гистерезис включен на 1 LSB");
    }else if(num_mode == '2') {
      Sensor.enableHysteresis2LSB(); // Включить на 2 LSB
      Serial.println("Гистерезис включен на 2 LSB");
    }else if(num_mode == '3') {
      Sensor.enableHysteresis3LSB(); // Включить на 3 LSB
      Serial.println("Гистерезис включен на 3 LSB");
    }else {
      Serial.println("Ошибка! Можно вводить только: 0, 1, 2, 3"); // Если отправленное значение режима не 0, 1, 2, 3
    }
  }
}

/*
 * Другие методы:
 * AS5600Hysteresis getHysteresis(); // Получить параметры гистерезиса
 * void setHysteresis(_hysteresis); // Установить новые параметры гистерезиса
 * bool setHysteresisVerify(_hysteresis); // с подтверждением
 * 
 * _hysteresis:
 * AS5600_HYSTERESIS_OFF
 * AS5600_HYSTERESIS_1_LSB
 * AS5600_HYSTERESIS_2_LSB
 * AS5600_HYSTERESIS_3_LSB
 * 
 * с проверкой:
 * bool disableHysteresisVerify();
 * bool enableHysteresis1LSBVerify();
 * bool enableHysteresis2LSBVerify();
 * bool enableHysteresis3LSBVerify();
 */
