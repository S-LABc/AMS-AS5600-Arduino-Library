# AMS-AS5600-Arduino-Library ✨
<p align="center"><img src="/images/as5600_aliexpress.jpg" width="39%"><img src="/images/assembly_case_stl.jpg" width="30%"></p>

## 📃 Общие сведения
* Полная и удобная библиотека Arduino IDE для работы с магнитными датчиками положения AS5600 и AS5600L.
* Поддерживаются все функции датчиков
* [Устанавливается](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki/%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0) как обычная библиотека для Arduino IDE.
* Справочный раздел [Wiki](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki)
* Использует библиотеку **Wire.h** для работы с шиной I²C
* Поддерживается большинством плат: *STM32, AVR, ESP8266, ESP32 ...*
* Имеет методы с верифкацией (проверкой выполнения)
* Имеет методы со ссылками (references)
* Имеет программную кнопку (экспериментально, не является стандартным решением)
* [3D-модель](addons/AS5600-Case-STL/) корпуса для макета
* Куча встроенных [примеров](examples/): *упрвление громкостью, [несколько датчиков](examples/AnyMCU/MultiSensorsTCA9548A_Serial), упрвление яркостью LED, Bluetooth на ESP32, [AS5600 Visually](addons/AS5600-Visually/), дисплей SSD1306, сервер на ESP32 ...*
* Класс AS5600L наследует все методы AS5600 с переопределением некоторых и расширяет его своими. Подробнее в [примерах](examples/) и [Wiki](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki)
* В AS5600L можно искать датчик на шине, если он один и не известен его адрес

## 💡 Дополнительно
* Библиотека для **магнитного энкодера** [AS5601](https://github.com/S-LABc/AMS-AS5601-Arduino-Library)
* Исходный код приложения [AS5600 Visually](https://github.com/S-LABc/AS5600-Position-Sensor-UI)
* Исходники [корпуса](https://github.com/S-LABc/AS5600-Case)
* Миниатюрная [печатная плата](https://github.com/S-LABc/AMS-AS5600-AS5601-Sensors-Board)

## ✌️ Ссылки
* [Даташит AS5600](https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf)
* [Даташит AS5600L](https://look.ams-osram.com/m/657fca3b775890b7/original/AS5600L-DS000545.pdf)
* [Страница AS5600](https://ams-osram.com/products/sensors/position-sensors/ams-as5600-position-sensor)
* [Страница AS5600L](https://ams-osram.com/products/sensors/position-sensors/ams-as5600l-magnetic-rotary-position-sensor)
* [Ядро ESP8266](https://github.com/esp8266/Arduino)
* [Ядро ESP32](https://github.com/espressif/arduino-esp32)
* [Ядро STM32 Роджера](https://github.com/rogerclarkmelbourne/Arduino_STM32)
* [Ядро STM32 официальное](https://github.com/stm32duino/Arduino_Core_STM32)
