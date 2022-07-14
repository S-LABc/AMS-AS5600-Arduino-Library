/*
 * GamepadXBox360_STM32
 * 
 * Пример эмуляции контроллера XBox360 с использованием датчика AS5600
 * 
 * Подключение датчика:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA (PB7)
 * SCL   -> SCL (PB6)
 *
 * Примечания:
 * 1. Скетч использует ядро Arduino_STM32 и библиотеку USBComposite_stm32f1
 * 2. Скетч настраивает микрокотроллер как контроллер XBox360
 *    используя класс USB HID XBox360
 * 3. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Нажать комбинацию Win+R
 * 4. В строку "Выполнить" ввести joy.cpl и нажать Enter
 * 5. В списке дважды кликнуть ЛКМ по "Controller (maple)"
 * 6. Приближать, отдалять, вращать магнит
 *
 * Проверка (другой способ):
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. В поиске ввести "панель управления" и нажать Enter
 * 4. В разделе "Оборудование и звук" выбрать "Просмотр устройств и принтеров"
 * 5. Кликнуть ПКМ по "USB Gamepad XBox360" и выбрать "Параметры игровых устройств управления"
 * 6. В списке дважды кликнуть ЛКМ по "Controller (maple)"
 * 7. Приближать, отдалять, вращать магнит
 * 
 * Зависимости:
 * https://github.com/rogerclarkmelbourne/Arduino_STM32
 * https://github.com/arpruss/USBComposite_stm32f1
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

// Подключаем библиотеки
#include <AMS_AS5600.h>
#include <USBComposite.h>

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

// Настройка USB как контроллера XBox360
USBXBox360 XBox360;

void setup() {
  // Устанавливаем общие сведения о USB устройстве
  USBComposite.setManufacturerString("S-LAB"); // Производитель устройства
  USBComposite.setProductString("USB Gamepad XBox360"); // Название устройства
  USBComposite.setSerialString("001"); // Серийный номер устройства

  // Запукаем геймпад
  XBox360.begin();

  // Запускаем соединение с датчиком
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Sensor.setClock(AS5600_I2C_CLOCK_100KHZ); // 100кГц
  //Sensor.setClock(AS5600_I2C_CLOCK_1MHZ); // 1МГц
  //Sensor.setClock(725000); // Пользовательское значение 725кГц

  // Ждем успешной инициализации USB
  while(!USBComposite);
}

void loop() {
  if(Sensor.isConnected()) { // Проверка на наличие связи с датчиком
	// Читаем положение магнита 0-4095
    word raw = Sensor.getRawAngle();
    // Приводим диапазона 0 - 4095 к диапазону -32767 - 32767
    int temp = map(raw, 0, 4095, -32767, 32767);
  
    // Положение по X/Y
    XBox360.position(temp, temp);
  }

  // Другие методы объекта XBox360
  //XBox360.button(uint8_t button, bool val);
  //XBox360.buttons(uint16_t b);
  //XBox360.X(int16_t val);
  //XBox360.Y(int16_t val);
  //XBox360.positionRight(int16_t x, int16_t y);
  //XBox360.XRight(int16_t val);
  //XBox360.YRight(int16_t val);
  //XBox360.sliderRight(uint8_t val);
  //XBox360.sliderLeft(uint8_t val);
}
