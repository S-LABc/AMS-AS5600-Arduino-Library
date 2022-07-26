/*
 * JoystickButtonUSB_STM32
 * 
 * Пример вращения вокруг одной из осей и
 * управление кнопкой USB джойстика с
 * использованием датчика AS5600
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
 * 2. Скетч настраивает микрокотроллер как джойстик
 *    используя класс USB HID Joystick
 * 3. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Нажать комбинацию Win+R
 * 4. В строку "Выполнить" ввести joy.cpl и нажать Enter
 * 5. В списке дважды кликнуть ЛКМ по "maple"
 * 6. Приближать, отдалять, вращать магнит
 *
 * Проверка (другой способ):
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. В поиске ввести "панель управления" и нажать Enter
 * 4. В разделе "Оборудование и звук" выбрать "Просмотр устройств и принтеров"
 * 5. Кликнуть ПКМ по "USB Joystick" и выбрать "Параметры игровых устройств управления"
 * 6. В списке дважды кликнуть ЛКМ по "maple"
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

// Номер кнопки у джойстика
#define JOYSTICK_BTN 3 // Диапазон 1-32, возможно бывает и более

// Значения AGC при перемещении магнита вдоль оси вращения. Подробнее в примере VirtualButton_Serial
#define MIN_AGC 99  // Минимальное значение возвращаемое методом getAutomaticGainControl()
#define MAX_AGC 115 // Максимальное значение возвращаемое методом getAutomaticGainControl()
#define DELTA_DIV 5 // Допустимое отклонение минимального и максимального значений

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

// Настройка USB как джойстика
USBHID HID;
HIDJoystick Joystick(HID);

void setup() {
  // Устанавливаем общие сведения о USB устройстве
  USBComposite.setManufacturerString("S-LAB"); // Производитель устройства
  USBComposite.setProductString("USB Joystick"); // Название устройства
  USBComposite.setSerialString("001"); // Серийный номер устройства
  
  // Запукаем джойстик
  HID.begin(HID_JOYSTICK);
  
  // Настройка границ срабатывания кнопки и отклонения. Подробнее в примере VirtualButton_Serial
  Sensor.setButtonMinAGC(MIN_AGC); // getButtonMinAGC()
  Sensor.setButtonMaxAGC(MAX_AGC); // getButtonMaxAGC()
  Sensor.setButtonDeviation(DELTA_DIV); // getButtonDeviation()
  
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
  // Если виртуальная кнопка датчика НАЖАТА
  if(Sensor.isButtonPressed()) {
    Joystick.button(JOYSTICK_BTN, true); // Отправляем сообщение с номером кнопки (3) и значением нажата (true)
  }

  // Если виртуальная кнопка датчика ОТПУЩЕНА
  if(Sensor.isButtonReleased()) {
    Joystick.button(JOYSTICK_BTN, false); // Отправляем сообщение с номером кнопки (3) и значением отпущена (false)
  }

  // Читаем положение магнита 0-4095
  word raw = Sensor.getRawAngle();
  // Приводим диапазона 0-4095 к диапазону 0-1023
  word temp = map(raw, 0, 4095, 0, 1023);
  
  // Вращение вокруг оси Y
  Joystick.Yrotate(temp);

  // Другие методы объекта Joystick, диапазон значений 0-1023
  //Joystick.X(x); // Положение по X
  //Joystick.Y(y); // Положение по Y
  //Joystick.Xrotate(x); // Поврот вокруг X
  //Joystick.Yrotate(y); // Поврот вокруг Y
  //Joystick.position(x,y); // Указание координат по X и Y
  //Joystick.sliderLeft(val); // Левый слайдер
  //Joystick.sliderRight(val); // Правый слайдер
  //Joystick.slider(val); // Оба слайдера
}
