# AMS-AS5600-Arduino-Library ✨
<p align="center"><img src="/images/as5600_aliexpress.jpg" width="39%"><img src="/images/assembly_case_stl.jpg" width="30%"></p>

## 📃 Общие сведения
* Полная и удобная библиотека Arduino IDE для работы с магнитным датчиком положения AS5600.
* Поддерживаются все функции датчика
* [Устанавливается](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki/%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0) как обычная библиотека для Arduino IDE.
* Справочный раздел [Wiki](https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki)
* Использует библиотеку **Wire.h** для работы с шиной I²C
* Поддерживается большинством плат: *STM32, AVR, ESP8266, ESP32 ...*
* Имеет методы с верифкацией (проверкой выполнения)
* Имеет программную кнопку (экспериментально, не является стандартным решением)
* [3D-модель](addons/AS5600-Case-STL/) корпуса для макета
* Куча встроенных [примеров](examples/): *упрвление громкостью, упрвление яркостью LED, Bluetooth на ESP32, [AS5600 Visually](addons/AS5600-Visually/), дисплей SSD1306, сервер на ESP32 ...*

## 💡 Дополнительно
* Библиотека для **магнитного энкодера** [AS5601](https://github.com/S-LABc/AMS-AS5601-Arduino-Library)
* Исходный код приложения [AS5600 Visually](https://github.com/S-LABc/AS5600-Position-Sensor-UI)
* Исходники [корпуса](https://github.com/S-LABc/AS5600-Case)
* Миниатюрная [печатная плата](https://github.com/S-LABc/AMS-AS5600-AS5601-Sensors-Board)
* Для работы с AS5600L можно написать класс унаследовав его от AS5600 с минимальными отличиями: можно менять адресс I²C (по умолчанию 0x40), отсутствуют оба режима аналогового выхода

## ✌️ Ссылки
* [Даташит AS5600](https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf)
* [Даташит AS5600L](https://ams.com/documents/20143/36005/AS5600L_DS000545_3-00.pdf)
* [Страница AS5600](https://ams.com/en/as5600)
* [Страница AS5600L](https://ams.com/as5600l)
* [Ядро ESP8266](https://github.com/esp8266/Arduino)
* [Ядро ESP32](https://github.com/espressif/arduino-esp32)
* [Ядро STM32 Роджера](https://github.com/rogerclarkmelbourne/Arduino_STM32)
* [Ядро STM32 официальное](https://github.com/stm32duino/Arduino_Core_STM32)
