/*
 * ReadRawAndDegreeAngle_SSD1306_I2C
 * 
 * Демонстрация вывода значений АЦП, угла в градусах
 * от датчика AS5600 на экран OLED_SSD1306_I2C
 * 
 * Подключение датчика:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Подключение дисплея:
 * SSD1306   Board
 * VCC    -> +3V3
 * GND    -> GND
 * SDA    -> SDA
 * SCL    -> SCL
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Подключить дисплей согласно распиновке
 * 3. Загрузить скетч в плату
 * 4. Менять положение магнита
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 * 
 * Зависимости:
 * https://github.com/adafruit/Adafruit_SSD1306
 * https://github.com/adafruit/Adafruit-GFX-Library
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

// Подключаем библиотеки
#include <AMS_AS5600.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Ширина и высота дисплея
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
// Адрес дисплея: 128х32 - 0x3C, для 128х64 может быть 0x3D
#define SCREEN_ADDRESS 0x3C
// Контакт сброса (если используется)
#define OLED_RESET 4
// Создаем объект display по стандартному методу
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Запускаем дисплей
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  /*
   * Sensor.begin() и Sensor.setClock() не используются
   * Adafruit_SSD1306 делает все это сама
   */
  
  // Если с датчиком порядок, настраивам дисплей для показаний угла
  displayInit();
}

void loop() {
  // Выводим новые данные на экран
  createText(Sensor.getRawAngle());
}

// Подготовка картики с новыми значениями
void createText(uint16_t angle_raw) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("ADC:");
  display.println(angle_raw);
  display.print("ANG:");
  display.print(angle_raw * 0.08789); // 360/4096=0,087890625, 5 знаков после точки для АЦП 12 бит достаточно
  //display.print(Sensor.getDegreesAngle()); // Можно получить угол сразу
  display.display();
}
// Очистка старого ИЗО, установка шрифта, цвета, курсора
void displayInit() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
}
