/*
 * MultiSensorsTCA9548A_Serial
 * 
 * Вывода значения угла в градусах от нескольких датчиков AS5600 в "Монитор порта"
 * с использованием мультиплексора шины I2C TCA9548A
 * 
 * Подключение:
 * Все контакты VCC -> +3V3 или +5V
 * Все контакты GND -> GND
 * Все контакты DIR -> GND
 * 
 * Мультиплексор
 * TCA9548A  board
 * A0       -> GND
 * A1       -> GND
 * A2       -> GND
 * RST      -> +3V3 или +5V
 * SDA      -> SDA
 * SCL      -> SCL
 * 
 * Первый датчик
 * AS5600  TCA9548A
 * SDA     -> SD0
 * SCL     -> SC0
 * 
 * Второй датчик
 * AS5600  TCA9548A
 * SDA     -> SD5
 * SCL     -> SC5
 * 
 * Проверка:
 * 1. Подключить датчики согласно распиновке
 * 2. Подключить мультиплексор согласно распиновке
 * 3. Загрузить скетч в плату
 * 4. Открыть "Монитор порта"
 * 5. Менять положение магнита
 * 6. Отправлять 0, 1, 2, 3, 4, 5, 6, 7 (на 0 и 5 будут значения двух датчиков)
 *
 * Примечания:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
 * Документация к мультиплексору:
 * https://www.ti.com/lit/ds/symlink/tca9548a.pdf
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

/*
 * Таблица выбора I2C адреса мультиплексора TCA9548A
 * 
 * A0  | A1  | A2  | TCA9548A_I2C_ADDRESS
 * --------------------------------------
 * GND | GND | GND | 0x70 (112)
 * --------------------------------------
 * VCC | GND | GND | 0x71 (113)
 * --------------------------------------
 * GND | VCC | GND | 0x72 (114)
 * --------------------------------------
 * VCC | VCC | GND | 0x73 (115)
 * --------------------------------------
 * GND | GND | VCC | 0x74 (116)
 * --------------------------------------
 * VCC | GND | VCC | 0x75 (117)
 * --------------------------------------
 * GND | VCC | VCC | 0x76 (118)
 * --------------------------------------
 * VCC | VCC | VCC | 0x77 (119)
 */
#define TCA9548A_I2C_ADDRESS 0x70 // 0x70 - 0x77 (112 - 119)

// Перечисление всех номеров шины (для удобства)
enum {
  TCA9548A_I2C_BUS_0, // 0
  TCA9548A_I2C_BUS_1, // 1
  TCA9548A_I2C_BUS_2, // 2
  TCA9548A_I2C_BUS_3, // 3
  TCA9548A_I2C_BUS_4, // 4
  TCA9548A_I2C_BUS_5, // 5
  TCA9548A_I2C_BUS_6, // 6
  TCA9548A_I2C_BUS_7  // 7
};

/*
 * Создаем объект Sensor с указанием ссылки на объект Wire
 * Объект должен быть один, т.к. адрес датчика не изменяется
 * Изменяется только номер шины мультиплексора
 */
AS5600 Sensor(&Wire);

void setup() {
  // Настраиваем "Монитор порта"
  Serial.begin(115200);
  
  /*
   * Запускаем и настраиваем шину I2C на 400кГц
   * В даном случаи действия выполняются через Wire,
   * а не через Sensor, т.к. шина общая для всех устройств
   */
  Wire.begin();
  Wire.setClock(AS5600_I2C_CLOCK_400KHZ);
  // Можно на друие частоты, но TCA9548A работает до 400кГц
  // AS5600_I2C_CLOCK_100KHZ 100кГц
  // AS5600_I2C_CLOCK_1MHZ 1МГц
  // 725000 Пользовательское значение 725кГц

  // Выбираем какую-нибудь начальную шину
  selectBusTCA9548A(TCA9548A_I2C_BUS_0);
  Serial.println("Датчик на шине I2C_BUS_0");
}

void loop() {
  // Проверка данных в буфере UART
  if (Serial.available()) {
    uint8_t num_sens = Serial.read(); // Чтение команды из "Монитора порта"

    // Переключение номера шины мультиплексора
    if (num_sens == '0') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_0);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_0");
    }
    else if (num_sens == '1') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_1);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_1");
    }
    else if (num_sens == '2') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_2);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_2");
    }
    else if (num_sens == '3') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_3);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_3");
    }
    else if (num_sens == '4') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_4);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_4");
    }
    else if (num_sens == '5') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_5);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_5");
    }
    else if (num_sens == '6') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_6);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_6");
    }
    else if (num_sens == '7') {
      selectBusTCA9548A(TCA9548A_I2C_BUS_7);
      Serial.println();
      Serial.println("Датчик на шине I2C_BUS_7");
    }
    else {
      Serial.println();
      Serial.println("Ошибка! Можно вводить только: 0 - 7"); // Если отправленное значение режима не 0, 1, 2, 3, 4, 5, 6, 7
      Serial.println();
    }
  }
  
  Serial.print("Угол в градусах: ");
  // Выводим значения в градусах (от 0 до 360)
  Serial.println(Sensor.getDegreesAngle());

  delay(100);
}

// Метод для переключения каналов (шин) мультиплексора
void selectBusTCA9548A(uint8_t bus_num) {
  // Запускаем соединение по адресу мультиплексора TCA9548A (A0, A1, A2)
  Wire.beginTransmission(TCA9548A_I2C_ADDRESS);
  // Устанавливаем бит отвечающий за номер шины выхода мультиплексора (I2C_BUS_0 - I2C_BUS_7)
  uint8_t bus = (1 << bus_num);
  // Отправляем номер шины в мультиплексор 
  Wire.write(bus);
  // Заканчиваем соединение
  Wire.endTransmission();
}
