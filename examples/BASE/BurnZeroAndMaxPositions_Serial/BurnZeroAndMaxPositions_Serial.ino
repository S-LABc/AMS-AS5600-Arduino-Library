/*
 * BurnZeroAndMaxPositions_Serial
 * 
 * Сохранение в энергонезависимую память датчика AS5600(AS5600L) значений
 * установленных методами setZeroPosition() и setMaxPosition()
 * 
 * ============ !!! ВАЖНО !!! ============
 * Этот скетч не проверялся на реальном датчике!
 * Метод burnZeroAndMaxPositions() не проверялся на реальном датчике!
 * Если значение ZMCO = 3, то выполнение метода burnZeroAndMaxPositions() вернет ошибку!
 * Если значение ZMCO отличается от 0, то выполнить burnMaxAngleAndConfigurationValue() можно частично, без записи MaxAngle!
 * Если Начальное и Конечное положения = 0, то записи не произойдет (по одному можно)!
 * Ответственность за все действия вы берете на себя!
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
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
 * 4. Следовать дальнейшим инструкциям
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
 * Copyright (C) 2024. v1.2 / Скляр Роман S-LAB
 */

// Подключаем библиотеку
#include <AMS_AS5600.h>

// УКАЖИТЕ ТУТ СВОИ ЗНАЧЕНИЯ УГЛОВ
#define Z_POS 800
#define M_POS 3469

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

  // Устанавливаем начальное и конечное положения
  Sensor.setZeroPosition(Z_POS);
  //Sensor.setZeroPositionViaRawAngle(); // Установить новое начальное положение ZPOS используя нынешнее положение магнита (getRawAngle)
  Sensor.setMaxPosition(M_POS);
  //Sensor.setMaxPositionViaRawAngle(); // Установить новое начальное положение MPOS используя нынешнее положение магнита (getRawAngle)

  Serial.println("!ВНИМАНИЕ!");
  Serial.println("Записать значения углов можно 3 раза!");
  Serial.println("После первой записи будет невозможно записать");
  Serial.println("MaxAngle методом burnMaxAngleAndConfigurationValue(),");
  Serial.println("но запись настроек сохранится!");
  Serial.println("");
  Serial.println("Максимальное количество записей в регистр ZMCO = 3");
  Serial.print("Сейчас значение регистра ZMCO = ");
  byte count = Sensor.getBurnPositionsCount();
  Serial.println(count);
  Serial.print("Возможно выполнить записать еще ");
  Serial.print(3 - count);
  Serial.println(" раз/раза");
  Serial.println("");
  Serial.println("Будут установены следующие значения:");
  Serial.print("Начальное положение (Z_POS) - ");
  Serial.println(Sensor.getZeroPosition());
  Serial.print("Конечное положение (M_POS) - ");
  Serial.println(Sensor.getMaxPosition());
  Serial.println("Для подтверждения отправьте символ Y");
  
  while(1) {
    byte c;
    if(Serial.available())
      c = Serial.read();
    if(c == 'Y') {
      break;
    }
    delay(100);
  }

  Serial.println();
  Serial.println("Принято! Запись значений углов началась");

  AS5600BurnReports report = Sensor.burnZeroAndMaxPositions();
  // Если не нужна заводская проверка записанного
  //AS5600BurnReports report = Sensor.burnZeroAndMaxPositions(AS5600_FLAG_SPECIAL_VERIFY_DISABLE);
  switch (report) {
    case AS5600_BURN_REPROT_SENSOR_NOT_CONNECTED:
      Serial.println("Датчик не обнаружен на шине I2C");
      break;
    case AS5600_BURN_REPROT_MAGNET_NOT_FOUND:
      Serial.println("Датчик не обнаружил магнит");
      break;
    case AS5600_BURN_REPROT_WRITE_OK:
      Serial.println("Начальный и конечный угол записаны (с проверкой)");
      break;
    case AS5600_BURN_REPROT_WRITE_WRONG:
      Serial.println("Начальный и конечный угол записать не удалось");
      break;
    case AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY:
      Serial.println("Начальный и конечный угол записаны (без проверки)");
      break;
    case AS5600_BURN_REPROT_ZPOS_MPOS_NOT_SET:
      Serial.println("Начальный или конечный угол не установлен");
      break;
    case AS5600_BURN_REPROT_ATTEMPTS_ENDED:
      Serial.println("Попытки для записи исчерпаны");
      break;
    default:
      Serial.println("Неизвестное значение отчета");
      break;
  }

  Serial.println("КОНЕЦ ПРОГРАММЫ");
}

void loop() {
  
}
