/*
 * VolumeLavelBluethoothBLE_ESP32
 *  
 * Демонстрация управления уровнем громкости в ОС
 * при помощи датчика AS5600 и микроконтроллера ESP32
 * 
 * Примечания:
 * 1. Скетч использует ядро arduino-esp32 и библиотеку ESP32-BLE-Keyboard
 * 2. Скетч настраивает микрокотроллер как мутимедийную клавиатуру
 *    используя класс BLE HID Consumer
 * 3. AS5600 не является инкрементальным энкодером, использовать его 
 *    таким образом, в реальных проектах, нежелтельно
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA (GPIO21)
 * SCL   -> SCL (GPIO22)
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Подключиться к устройству Bluetooth с именем BLE Multimedia
 * 3. Менять положение магнита
 * 4. Наблюдать изменение громкости в ОС
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 * 
 * Зависимости:
 * https://github.com/espressif/arduino-esp32
 * https://github.com/T-vK/ESP32-BLE-Keyboard
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
#include <BleKeyboard.h>

// Чувствительность на таймере
#define JOG_SENS 15 // Миллисекунды

// Сведения об устройстве
#define BLE_KEYBOARD_NAME           "BLE Multimedia" // Имя устройства
#define BLE_KEYBOARD_MANUFACTURER   "S-LAB" // Производитель
#define BLE_KEYBOARD_INIT_BAT_LAVEL 100 // Заряд АКБ, сейчас 100% из-за питания от USB кабеля

// Хранят значения для определения действия
uint16_t last_value = 0;
uint16_t now_value = 0;
int16_t delta_value = 0;
// Хранит время для регулировки чувствительности
uint32_t last_time = 0;

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

// Создаем объект MmultimediaKeyboardBLE с указанием имени устройства, названия производителя, уровнем заряда АКБ (если есть АКБ)
BleKeyboard MmultimediaKeyboardBLE(BLE_KEYBOARD_NAME, BLE_KEYBOARD_MANUFACTURER, BLE_KEYBOARD_INIT_BAT_LAVEL);

void setup() {
  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();

  // Запускаем BLE клавиатуру
  MmultimediaKeyboardBLE.begin();
}

void loop() {
  if(millis() - last_time > JOG_SENS) { // Регулировка чувствительности
    if(MmultimediaKeyboardBLE.isConnected() && Sensor.isConnected()) { // Проверка на наличие подключения к устройству и связи с датчиком
      sensorEvent(); // Опрос датчика
    }
    
    last_time = millis(); // Сохраняем время
  }
}

// Обработка показаний датчика и управление громкостью
// Тут можно найти другие коды клавиш https://github.com/T-vK/ESP32-BLE-Keyboard/blob/master/BleKeyboard.h#L35
void sensorEvent() {
  now_value = Sensor.getRawAngle(); // Получаем нынешнее значение (от 0 до 4095)
  delta_value = now_value - last_value; // Находим разность текущего и прошлого значений
  
  if(delta_value > 2) { // Если был поворот с разностью больше 2
    MmultimediaKeyboardBLE.write(KEY_MEDIA_VOLUME_UP); // Увеличиваем громкость
  }else if(delta_value < -2) { // Если был поворот с разностью меньше -2
    MmultimediaKeyboardBLE.write(KEY_MEDIA_VOLUME_DOWN); // Уменьшаем громкость
  }
  
  last_value = now_value; // Сохраняем нынешнее значение как прошлое
}

/*
 * Другие методы MmultimediaKeyboardBLE:
 * void end(void);
 * void sendReport(KeyReport* keys);
 * void sendReport(MediaKeyReport* keys);
 * 
 * size_t press(uint8_t k);
 * size_t press(const MediaKeyReport k);
 * 
 * size_t release(uint8_t k);
 * size_t release(const MediaKeyReport k);
 * void releaseAll(void);
 * 
 * size_t write(uint8_t c);
 * size_t write(const uint8_t *buffer, size_t size);
 * 
 * void setBatteryLevel(uint8_t level);
 * void setName(std::string deviceName);  
 * void setDelay(uint32_t ms);
 * 
 * void set_vendor_id(uint16_t vid);
 * void set_product_id(uint16_t pid);
 * void set_version(uint16_t version);
 */
