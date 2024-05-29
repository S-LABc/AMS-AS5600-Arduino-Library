/*
 * ChangeVolumeOS_STM32
 *  
 * Демонстрация управления уровнем громкости в ОС
 * при помощи датчика AS5600 и микроконтроллера STM32F1
 * 
 * Примечания:
 * 1. Скетч использует ядро Arduino_STM32 и библиотеку USBComposite_stm32f1
 * 2. Скетч настраивает микрокотроллер как мутимедийную клавиатуру
 *    используя класс USB HID Consumer
 * 3. AS5600 не является инкрементальным энкодером, использовать его 
 *    таким образом, в реальных проектах, нежелтельно
 * 4. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA (PB7)
 * SCL   -> SCL (PB6)
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Менять положение магнита
 * 4. Наблюдать изменение громкости в ОС
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
 * 
 * Зависимости:
 * https://github.com/rogerclarkmelbourne/Arduino_STM32
 * https://github.com/arpruss/USBComposite_stm32f1
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

// Подключаем библиотеки
#include <AMS_AS5600.h>
#include <USBComposite.h>

// Коды кнопок изменения громкости
// Больше кодов клавиш можно найти тут https://github.com/NicoHood/HID/blob/master/src/HID-APIs/ConsumerAPI.h#L30
#define VOLUME_UP   0xE9
#define VOLUME_DOWN 0xEA

// Чувствительность на таймере
#define JOG_SENS 15 //миллисекунды

// Хранят значения для определения действия
uint16_t last_value = 0;
uint16_t now_value = 0;
int16_t delta_value = 0;
// Хранит время для регулировки чувствительности
uint32_t last_time = 0;

// Настройка USB как мультимедийной клавиатуры
USBHID HID;
const uint8_t reportDescription[] = {
   HID_CONSUMER_REPORT_DESCRIPTOR()
};
HIDConsumer MmultimediaKeyboard(HID);

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Устанавливаем общие сведения о USB устройстве
  USBComposite.setManufacturerString("S-LAB"); // Производитель устройства
  USBComposite.setProductString("USB Multimedia Keyboard"); // Название устройства
  USBComposite.setSerialString("001"); // Серийный номер устройства
  
  // Запукаем клавиатуру
  HID.begin(reportDescription, sizeof(reportDescription));

  // Запускаем соединение
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
  // Регулировка чувствительности
  if(millis() - last_time > JOG_SENS) {
    sensorEvent(); // Опрос датчика
    last_time = millis(); // Сохраняем время
  }
}
// Обработка показаний датчика и управление громкостью
void sensorEvent() {
  if(Sensor.isConnected()) {
    now_value = Sensor.getRawAngle(); // Получаем нынешнее значение (от 0 до 4095)
    delta_value = now_value - last_value; // Находим разность текущего и прошлого значений
  
    if(delta_value > 2) { // Если был поворот с разностью больше 2
      MmultimediaKeyboard.press(VOLUME_UP); // Увеличиваем громкость
      MmultimediaKeyboard.release(); // Отпускаем нажатую клавишу
    }else if(delta_value < -2) { // Если был поворот с разностью меньше -2
      MmultimediaKeyboard.press(VOLUME_DOWN); // Уменьшаем громкость
      MmultimediaKeyboard.release(); // Отпускаем нажатую клавишу
    }

    last_value = now_value; // Сохраняем нынешнее значение как прошлое
  }
}
