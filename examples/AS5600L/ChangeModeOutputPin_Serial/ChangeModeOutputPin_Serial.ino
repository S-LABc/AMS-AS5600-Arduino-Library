/*
 * ChangeModeOutputPin_Serial
 *  
 * Демонстрация изменения режима работы контакта OUT датчика AS5600L
 * путем отправление команд через "Монитор порта"
 * 
 * Примечания:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 2. Датчика AS5600L не имеет аналогового выхода, только ШИМ
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
 * 4. Отправлять 0, 1, или 2
 * 5. Менять положение магнита
 * 6. Замерять частоту ШИМ сигнала
 *    на контакте OUT датчика
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/657fca3b775890b7/original/AS5600L-DS000545.pdf
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
#include <AMS_AS5600L.h>

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600L Sensor(&Wire);

void setup() {
  Serial.begin(115200);
  
  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 100кГц
  Sensor.setClock(AS5600_I2C_CLOCK_100KHZ);
}

void loop() {
  // Проверка данных в буфере UART
  if(Serial.available()) {
    uint8_t num_mode = Serial.read(); // Чтение команды из "Монитора порта"

    // Установка разных режимов
    if(num_mode == '0') {
      Sensor.enableOutputAnalogFullRange(); // Для AS5600L этот режим подтягивает OUT к VDD
      Serial.println("Режим: Аналоговый полный 0-100% НЕДОСТУПЕН ДЛЯ AS5600L");
    }else if(num_mode == '1') {
      Sensor.enableOutputAnalogReducedRange(); // Для AS5600L этот режим подтягивает OUT к VDD
      Serial.println("Режим: Аналоговый сжатый 10-90% НЕДОСТУПЕН ДЛЯ AS5600L");
    }else if(num_mode == '2') {
      Sensor.enableOutputDigitalPWM(); // Просто цифровой ШИМ сигнал
      Serial.println("Режим: Цифровой ШИМ");
    }else {
      Serial.println("Ошибка! Можно вводить только: 0, 1, 2"); // Если отправленное значение режима не 0, 1, 2
    }
  }
}

/*
 * Все методы управления контактом OUT
 * 
 * AS5600OutputStage getOutputStage(); // Получить режим работы контакта OUT
 * void setOutputStage(AS5600OutputStage _output_stage); // Установить режим работы контакта OUT
 * bool setOutputStageVerify(AS5600OutputStage _output_stage); // с подтверждением
 * 
 * _output_stage:
 * AS5600_OUTPUT_ANALOG_FULL_RANGE - недоступен для AS5600L
 * AS5600_OUTPUT_ANALOG_REDUCED_RANGE - недоступен для AS5600L
 * AS5600_OUTPUT_DIGITAL_PWM - доступен для AS5600L
 * 
 * void enableOutputAnalogFullRange(); // OUT как аналоговый выход (0 - 100%) НЕДОСТУПЕН ДЛЯ AS5600L
 * bool enableOutputAnalogFullRangeVerify(); // с подтверждением
 * 
 * void enableOutputAnalogReducedRange(); // OUT как аналоговый выход (10 - 90%) НЕДОСТУПЕН ДЛЯ AS5600L
 * bool enableOutputAnalogReducedRangeVerify(); // с подтверждением
 * 
 * void enableOutputDigitalPWM(); // OUT как цифровой ШИМ выход
 * bool enableOutputDigitalPWMVerify(); // с подтверждением
 */
