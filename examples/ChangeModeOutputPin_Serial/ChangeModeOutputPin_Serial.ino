/*
 * ChangeModeOutputPin_Serial
 *  
 * Демонтрация изменения режима работы контакта OUT датчика AS5600
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
 * 4. Отправлять 1, 2, или 3
 * 5. Менять положение магнита
 * 4. Замерять напряжение или
 *    частоту ШИМ сигнала на 
 *    контакте OUT датчика
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
}

void loop() {
  // Проверка данных в буфере UART
  if(Serial.available()) {
    uint8_t num_mode = Serial.read(); // Чтение команды из "Монитора порта"

    // Установка разных режимов
    if(num_mode == '0') {
      Sensor.enableOutputAnalogFullRange(); // Изменение напряжения от 0% до 100% между GND и VDD
      Serial.println("Mode: Analog Full Range");
    }else if(num_mode == '1') {
      Sensor.enableOutputAnalogReducedRange(); // Изменение напряжения от 10% до 90% между GND и VDD
      Serial.println("Mode: Analog Reduced Range");
    }else if(num_mode == '2') {
      Sensor.enableOutputDigitalPWM(); // Просто цифровой ШИМ сигнал
      Serial.println("Mode: Digital PWM");
    }else {
      Serial.println("Error! Send 0, 1, 2"); // Если отправленное значение режима не 0, 1, 2
    }
  }
}
