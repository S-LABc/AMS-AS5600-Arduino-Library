/*
 * AngleToHtmlServer_ESP32
 * 
 * Демонстрация вывода данных от датчика AS5600 на веб страницу браузера
 * с использовнием платформы ESP32
 * 
 * Подключение:
 * AS5600   ESP32
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> GPIO21
 * SCL   -> GPIO22
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату модуля
 * 3. Открыть список WiFi сетей
 * 4. Подключиться к сети ESP32 с паролем 01234567
 * 5. Перейти в браузере по адресу 192.168.2.1
 * 6. Менять положение магнита и нажимать кнопку на странице
 * 
 * Кусок кода с сервером и страницей взят отсюда:
 * https://arduino-tex.ru/news/15/urok-1-veb-server-esp32-esp8266-v-srede-arduino-ide.html
 * 
 * Документация к датчику:
 * https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 *
 * Зависимости:
 * https://github.com/espressif/arduino-esp32
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
 * Copyright (C) 2022. v1.1 / Скляр Роман S-LAB
 */

// Подключаем библиотеки
#include <AMS_AS5600.h>
#include <WiFi.h>
#include <WebServer.h>

// Имя сети и пароль
const char* ssid = "ESP32";
const char* password = "01234567";
// Настройки IP адреса
IPAddress local_ip(192,168,2,1); // Адрес который надо ввести в браузер
IPAddress gateway(192,168,2,1);
IPAddress subnet(255,255,255,0);
WebServer server(80);

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Запускаем соединение с датчиком
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Sensor.setClock(AS5600_I2C_CLOCK_100KHZ); // 100кГц
  //Sensor.setClock(AS5600_I2C_CLOCK_1MHZ); // 1МГц
  //Sensor.setClock(725000); // Пользовательское значение 725кГц

  // Настройки точки доступа
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  // Обработчик главной директории
  server.on("/", handle_main);
  // Запускаем сервер
  server.begin();
}

void loop() {
  // Обработчик запросов от браузера
  server.handleClient();
}

// Обработчик корневой директории
void handle_main() {
  server.send(200, "text/html", sendHTML(Sensor.getRawAngle())); // Отправляем угол на страницу и передаем ее в браузер
}

// Веб страница
String sendHTML(uint16_t _raw_ang){
  String html = "<!DOCTYPE html> <html>\n";
  html +="<meta http-equiv=\"Content-type\" content=\"text/html; charset=utf-8\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  html +="<title>AS5600 Web UI</title>\n";
  html +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  html +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  html +=".button {display: block;width: 80px;background-color: #3498db;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 18px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  html +=".btn {background-color: #34495e;}\n";
  html +=".btn:active {background-color: #2c3e50;}\n";
  html +="p {font-size: 24px;color: #888;margin-bottom: 10px;}\n";
  html +="</style>\n";
  html +="</head>\n";
  html +="<body>\n";
  html +="<h2>AMS AS5600</h2>\n";
  html +="<div>Обновляйте страницу, меняя положение магнита</div>\n";
  html +="<p>АЦП: " + String(_raw_ang) + "</p>";
  html +="<p>Угол: " + String(_raw_ang * 0.08789) + "</p>"; // 360/4096=0,087890625, 5 знаков после точки для АЦП 12 бит достаточно
  // Вместо _raw_ang*0.08789 можно так Sensor.getDegreesAngle()
  html +="<a class=\"button btn\" href=\"/\">Обновить</a>\n";
  html +="</body>\n";
  html +="</html>\n";
  
  return html;
}
