/*
 * ChangeModeOutputPin_Serial
 *  
 * Демонстрация изменения режима работы контакта OUT датчика AS5600
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
 * 4. Отправлять 0, 1, или 2
 * 5. Менять положение магнита
 * 4. Замерять напряжение или
 *    частоту ШИМ сигнала на 
 *    контакте OUT датчика
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
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
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
      Sensor.enableOutputAnalogFullRange(); // Изменение напряжения от 0% до 100% между GND и VDD
      Serial.println("Режим: Аналоговый полный 0-100%");
    }else if(num_mode == '1') {
      Sensor.enableOutputAnalogReducedRange(); // Изменение напряжения от 10% до 90% между GND и VDD
      Serial.println("Режим: Аналоговый сжатый 10-90%");
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
 * AS5600_OUTPUT_ANALOG_FULL_RANGE
 * AS5600_OUTPUT_ANALOG_REDUCED_RANGE
 * AS5600_OUTPUT_DIGITAL_PWM
 * 
 * void enableOutputAnalogFullRange(); // OUT как аналоговый выход (0 - 100%)
 * bool enableOutputAnalogFullRangeVerify(); // с подтверждением
 * 
 * void enableOutputAnalogReducedRange(); // OUT как аналоговый выход (10 - 90%)
 * bool enableOutputAnalogReducedRangeVerify(); // с подтверждением
 * 
 * void enableOutputDigitalPWM(); // OUT как цифровой ШИМ выход
 * bool enableOutputDigitalPWMVerify(); // с подтверждением
 */
