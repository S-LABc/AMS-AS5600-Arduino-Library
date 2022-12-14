/*
 * burnMaxAngleAndConfigurationValue_Serial
 * 
 * Сохранение в энергонезависимую память датчика AS5600(AS5600L) значений
 * установленных методами setMaxAngle() и setRawConfigurationValue()
 * 
 * ============ !!! ВАЖНО !!! ============
 * Этот скетч не проверялся на реальном датчике!
 * Метод burnMaxAngleAndConfigurationValue() не проверялся на реальном датчике!
 * Если значение ZMCO > 0, то выполнение метода burnMaxAngleAndConfigurationValue() запишет только конфигурации!
 * Если значение ZMCO = 0, то выполнить burnMaxAngleAndConfigurationValue() можно полностью!
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
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
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
#include <AMS_AS5600.h>

// УКАЖИТЕ ТУТ СВОИ ЗНАЧЕНИЯ
#define R_CONFIG 0x1745
#define M_ANGLE 1627

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

  // Устанавливаем значение максимального угла
  Sensor.setMaxAngle(M_ANGLE);
  //Sensor.setMaxAngleViaRawAngle(); // Установить новое начальное положение MANG используя нынешнее положение магнита (getRawAngle)
  // Устанавливаем значение конфигураций
  Sensor.setRawConfigurationValue(R_CONFIG);
  // Можно устанавливать отдельными методами, например:
  // setPowerMode(), enableLowPowerMode1(), enablePWMFrequency920Hz ...

  Serial.println("!ВНИМАНИЕ!");
  Serial.println("Записать значения можно 1 раз!");
  Serial.println("Если ZMCO > 0, то запишутся только конфигурации");
  Serial.println("Если ZMCO = 0, то запишется все");
  Serial.println("");
  Serial.print("Сейчас значение регистра ZMCO = ");
  byte count = Sensor.getBurnPositionsCount();
  Serial.println(count);
  Serial.println("");
  Serial.println("Будут установены следующие значения:");
  Serial.print("Макимальный угол - ");
  Serial.println(Sensor.getMaxAngle());
  Serial.print("Конфигурации - ");
  Serial.println(Sensor.getRawConfigurationValue());
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
  Serial.println("Принято! Запись значений началась");

  AS5600BurnReports report = Sensor.burnMaxAngleAndConfigurationValue();
  // Если не нужна заводская проверка записанного
  //AS5600BurnReports report = Sensor.burnMaxAngleAndConfigurationValue(AS5600_FLAG_SPECIAL_VERIFY_DISABLE);
  switch (report) {
    case AS5600_BURN_REPROT_SENSOR_NOT_CONNECTED:
      Serial.println("Датчик не обнаружен на шине I2C");
      break;
    case AS5600_BURN_REPROT_MAGNET_NOT_FOUND:
      Serial.println("Датчик не обнаружил магнит");
      break;
    case AS5600_BURN_REPROT_WRITE_OK:
      Serial.println("Макимальный угол и конфигурации записаны (с проверкой)");
      break;
    case AS5600_BURN_REPROT_WRITE_WRONG:
      Serial.println("Макимальный угол и конфигурации записать не удалось");
      break;
    case AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY:
      Serial.println("Макимальный угол и конфигурации записаны (без проверки)");
      break;
    case AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL:
      Serial.println("Значение максимального угла меньше AS5600_MIN_ANGLE_VALUE_DEC");
      break;
    case AS5600_BURN_REPROT_WRITE_OK_WITHOUT_MAXANGLE:
      Serial.println("Записаны только конфигурации (с проверкой)");
      break;
    case AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY_WITHOUT_MAXANGLE:
      Serial.println("Записаны только конфигурации (без проверки)");
      break;
    default:
      Serial.println("Неизвестное значение отчета");
      break;
  }

  Serial.println("КОНЕЦ ПРОГРАММЫ");
}

void loop() {
  
}
