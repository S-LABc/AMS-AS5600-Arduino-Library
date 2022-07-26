/*
 * GamepadButtonBluethoothBLE_ESP32
 *  
 * Демонстрация вращения вокруг оси Z Bluetooth геймпадом
 * при помощи датчика AS5600 и микроконтроллера ESP32
 * 
 * Примечание:
 * Скетч использует ядро arduino-esp32, библиотеки ESP32-BLE-Gamepad и NimBLE-Arduino
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
 * 3. Нажать комбинацию Win+R
 * 4. В строку "Выполнить" ввести joy.cpl и нажать Enter
 * 5. В списке дважды кликнуть ЛКМ по устройству
 * 6. Приближать, отдалять, вращать магнит
 *
 * Проверка (другой способ):
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. В поиске ввести "панель управления" и нажать Enter
 * 4. В разделе "Оборудование и звук" выбрать "Просмотр устройств и принтеров"
 * 5. Кликнуть ПКМ по "BLE Gamepad" и выбрать "Параметры игровых устройств управления"
 * 6. В списке дважды кликнуть ЛКМ по устройству
 * 7. Приближать, отдалять, вращать магнит
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 * 
 * Зависимости:
 * https://github.com/espressif/arduino-esp32
 * https://github.com/lemmingDev/ESP32-BLE-Gamepad
 * https://github.com/h2zero/NimBLE-Arduino
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
#include <BleGamepad.h>

// Сведения об устройстве
#define BLE_KEYBOARD_NAME           "BLE Gamepad" // Имя устройства
#define BLE_KEYBOARD_MANUFACTURER   "S-LAB" // Производитель
#define BLE_KEYBOARD_INIT_BAT_LAVEL 100 // Заряд АКБ, сейчас 100% из-за питания от USB кабеля

// Значения AGC при перемещении магнита вдоль оси вращения. Подробнее в примере VirtualButton_Serial
#define MIN_AGC 99  // Минимальное значение возвращаемое методом getAutomaticGainControl()
#define MAX_AGC 115 // Максимальное значение возвращаемое методом getAutomaticGainControl()
#define DELTA_DIV 5 // Допустимое отклонение минимального и максимального значений

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

// Создаем объект GamepadBLE с указанием имени устройства, названия производителя, уровнем заряда АКБ (если есть АКБ)
BleGamepad GamepadBLE(BLE_KEYBOARD_NAME, BLE_KEYBOARD_MANUFACTURER, BLE_KEYBOARD_INIT_BAT_LAVEL);

void setup() {
  // Настройка границ срабатывания кнопки и отклонения. Подробнее в примере VirtualButton_Serial
  Sensor.setButtonMinAGC(MIN_AGC); // getButtonMinAGC()
  Sensor.setButtonMaxAGC(MAX_AGC); // getButtonMaxAGC()
  Sensor.setButtonDeviation(DELTA_DIV); // getButtonDeviation()
  
  // Запускаем соединение с датчиком
  Sensor.begin();
  // Настраиваем шину I2C на 1МГц
  Sensor.setClock(AS5600_I2C_CLOCK_1MHZ);

  // Запускаем BLE геймпад
  GamepadBLE.begin();
}

void loop() {
  if(GamepadBLE.isConnected() && Sensor.isConnected()) { // Проверка на наличие подключения к устройству и связи с датчиком
    // Получаем нынешнее значение (от 0 до 4095)
    word now_value = Sensor.getRawAngle();
    // Приводим диапазона 0 - 4095 к диапазону -32767 - 32767
    int remap_value = map(now_value, 0, 4095, -32767, 32767);

    // Все методы объекта GamepadBLE https://github.com/lemmingDev/ESP32-BLE-Gamepad/blob/master/BleGamepad.h#L49
    // Некоторые значения аргуметов https://github.com/lemmingDev/ESP32-BLE-Gamepad/blob/master/BleGamepadConfiguration.h#L13
    
    GamepadBLE.setRZ(remap_value); // Перемещаемся вокруг оси Z

    // Если виртуальная кнопка датчика НАЖАТА
    if(Sensor.isButtonPressed()) {
      GamepadBLE.press(BUTTON_4); // Отправляем сообщение с номером кнопки (4)
    }

    // Если виртуальная кнопка датчика ОТПУЩЕНА
    if(Sensor.isButtonReleased()) {
      GamepadBLE.release(BUTTON_4); // Отправляем сообщение с номером кнопки (4)
    }
  }
}
