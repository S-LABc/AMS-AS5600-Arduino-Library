/*
 * ChangeDirection_Serial
 * 
 * Демонстрация смены положительного направления вращения датчика AS5600
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> PIN_BOARD
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
 * 4. Смотреть как меняются значения
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
 * Copyright (C) 2022. v1.2 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

/* 
 * Создаем объект Sensor с указанием ссылки на объект Wire
 * и вывода микроконтроллера куда подключен контакт DIR датчика
 * при необходимости можно указать направление (по умолчнаю по часовой стрелке)
 * 
 * STM32_AS5600_DEF_PIN -> PC13
 * ESP32_AS5600_DEF_PIN -> 4
 * ARDUINO_AS5600_DEF_PIN -> 3
 */
AS5600 Sensor(&Wire, STM32_AS5600_DEF_PIN);

// Тоже самое с указанием положительного направления против часовой стрелки
//AS5600 Sensor(&Wire, STM32_AS5600_DEF_PIN, AS5600_DIRECTION_POLARITY_COUNTERCLOCKWISE);

// Тоже самое с указанием положительного направления по часовой стрелке
//AS5600 Sensor(&Wire, STM32_AS5600_DEF_PIN, AS5600_DIRECTION_POLARITY_CLOCKWISE);

// Для хранения времени
uint32_t last_time = 0;

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

  /* setDirection() и reverseDirection() работают только 
   * если указан контакт МК.
   * Можно указать через метод Sensor.attachDirectionPin(byte _pin_dir);
   */
  // Положительное направление против часовой стрелки
  //Sensor.setDirection(AS5600_DIRECTION_POLARITY_COUNTERCLOCKWISE);
  // Положительное направление по часовой стрелке
  //Sensor.setDirection(AS5600_DIRECTION_POLARITY_CLOCKWISE); 
}

void loop() {
  if(millis() - last_time > 5000) { // Если прошло больше пяти секунд
    Sensor.reverseDirection(); // Сменить направление на противоположное
    
    if(Sensor.getDirection()) { // Если 1, против часовой стрелки
      Serial.println();
      Serial.println("Направление: Против часовой стрелки");
    }else { // Если 0, по часовой стрелке
      Serial.println();
      Serial.println("Направление: По часовой стрелке");
    }
    
    last_time = millis(); // Запомнить время
  }
  
  Serial.println(Sensor.getRawAngle()); // Выводим текущее значение угла в формате АЦП
  delay(100);
}
