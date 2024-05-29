/*
 * AngleToHtmlServerAJAX_ESP32
 * 
 * Демонстрация вывода угла от датчика AS5600 на веб страницу браузера
 * с использовнием платформы ESP32 без обновления страницы вручную
 * с асинхронным сервером обработки запросов
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
 * 6. Менять положение магнита
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
 *
 * Зависимости:
 * https://github.com/espressif/arduino-esp32
 * https://github.com/me-no-dev/AsyncTCP
 * https://github.com/me-no-dev/ESPAsyncWebServer
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

// Подключаем библиотеки
#include <AMS_AS5600.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Главная веб-страница в виде массива из символов сохраненного в памяти
static const char PAGE_index[] PROGMEM = R"=====(<!DOCTYPE html><html lang="ru">
<meta http-equiv="Content-type" content="text/html; charset=utf-8"><head><meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
<title>AS5600 Web UI AJAX</title><style>html{font-family:Helvetica;display:inline-block;margin:0px auto;text-align:center;}
body{margin-top:50px;}</style></head><body><h2>AMS AS5600</h2><div>Угол в градусах: <span id="Angle">0</span></div><br><br>
<script>setInterval(function(){getAngle();},500);function getAngle(){var AngleRequest=new XMLHttpRequest();AngleRequest.onreadystatechange=function(){
if(this.readyState==4&&this.status==200){document.getElementById("Angle").innerHTML=this.responseText;}};
AngleRequest.open("GET","angle",true);AngleRequest.send();}</script></body></html>)=====";

// Имя сети и пароль
const char* ssid = "ESP32";
const char* password = "01234567";

// Настройки IP адреса
IPAddress local_ip(192,168,2,1); // Адрес который надо ввести в браузер
IPAddress gateway(192,168,2,1);
IPAddress subnet(255,255,255,0);

// Объект асинхронного сервера
AsyncWebServer server(80);

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  // Запускаем соединение с датчиком
  Sensor.begin();
  // Можно указать выводы для I2C, SDA=33 SCL=32
  //Sensor.begin(33, 32);
  
  // Настраиваем шину I2C на 1МГц
  Sensor.setClock(AS5600_I2C_CLOCK_1MHZ);
  //Sensor.setClock(AS5600_I2C_CLOCK_100KHZ); // 100кГц
  //Sensor.setClock(AS5600_I2C_CLOCK_400KHZ); // 400кГц
  //Sensor.setClock(); // Тоже 400кГц
  //Sensor.setClock(725000); // Пользовательское значение 725кГц

  // Настройки точки доступа
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(500);
  
  // Обработчик главной директории
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", PAGE_index);
  });
  // Обработчик запроса угла
  server.on("/angle", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", String(Sensor.getDegreesAngle()));
  });
  
  // Запускаем асинхронный сервер
  server.begin();
}

void loop() {}
