/*
 * SensorUSBHID_STM32
 * 
 * Демонстрация вывода данных от датчика AS5600 на компьютер
 * для отображения в скрипте Python "SensorHID.py"
 * 
 * Подключение:
 * AS5600   Board
 * VCC      -> +3V3
 * GND      -> GND
 * DIR      -> GND
 * SDA      -> SDA
 * SCL      -> SCL
 * 
 * Примечения:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 2. Скетч использует ядро Arduino_STM32 и библиотеку USBComposite_stm32f1
 * 3. Скетч настраивает микрокотроллер USB HID устройство без классификации
 * 4. Запущенное устройство не требует никаких драйверов
 *
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Запустить скрипт Python [библиотеки]\AMS-AS5600-Arduino-Library\addons\AS5600-Sensor-HID\SensorHID.py
 * 4. Смотреть расшифровку принятых данных в терминале Python
 * 
 * Зависимости:
 * https://github.com/rogerclarkmelbourne/Arduino_STM32
 * https://github.com/arpruss/USBComposite_stm32f1
 * 
 * Документация к датчику:
 * AS5600  https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
 * AS5600L https://look.ams-osram.com/m/657fca3b775890b7/original/AS5600L-DS000545.pdf
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
//#include <AMS_AS5600L.h> // Для другой версии датчика
#include <USBComposite.h>

// Раскомментировать, если используется второй аппаратный блок I2C у платы
//TwoWire Wire2 (2, I2C_FAST_MODE);
//#define Wire Wire2

#define TX_SIZE 23 // Количество передаваемых байт данных по USB
#define RX_SIZE 1 // Количество принимаемых байт данных по USB за вычетом 1 байта Report ID, так требует библиотека USBComposite

// Общая информация об устройстве
#define DEV_VID   0x1EAF // Значение по умолчанию (связано со скриптом Python)
#define DEV_PID   0x0024 // Значение по умолчанию (связано со скриптом Python)
#define DEV_MUNUF "S-LAB"
#define DEV_PROD  "SENSOR-HID-RAW"
#define DEV_SER   "v001"

#define UNIQ_KEY 0x15 // Уникальный ключ (связан со скриптом Python)

// Инициализация массивов буферизации для принимаемых и передаваемых данных
uint8_t rx_buf[RX_SIZE];
uint8_t tx_buf[TX_SIZE];

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

// Создание объекта HID плагина
USBHID HID;
// Создание объекта чистого HID
HIDRaw<TX_SIZE, RX_SIZE> rawHID(HID);
// Дескриптор отчета для чистого HID
const uint8_t reportDescription[] = {
  HID_RAW_REPORT_DESCRIPTOR(TX_SIZE, RX_SIZE)
};

void setup(){
  // Настройка информации об устройстве
  USBComposite.setVendorId(DEV_VID);
  USBComposite.setProductId(DEV_PID);
  USBComposite.setManufacturerString(DEV_MUNUF);
  USBComposite.setProductString(DEV_PROD);
  USBComposite.setSerialString(DEV_SER);
  
  // Зпуск плагина HID
  HID.begin(reportDescription, sizeof(reportDescription));
  // Запускаем соединение с датчиком
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Sensor.setClock(AS5600_I2C_CLOCK_100KHZ); // 100кГц
  //Sensor.setClock(AS5600_I2C_CLOCK_1MHZ); // 1МГц
  //Sensor.setClock(725000); // Пользовательское значение 725кГц
  
  // Ожидание установки связи по USB
  while (!USBComposite);
  // Запуск чистого HID
  rawHID.begin();
}

void loop() {
  if (rawHID.getOutput(rx_buf)) { // Если пришли данные от хоста
    if (rx_buf[0] == UNIQ_KEY) { // Если в данных есть правильный уникальный ключ
      // Собираем данные в буфер для передачи
      prepareData();
      // Отправляем собранные данные хосту
      rawHID.send(tx_buf, sizeof(tx_buf));
    }
  }
}

void prepareData() {
  tx_buf[0] = Sensor.isConnected();
      
  word zero_pos = Sensor.getZeroPosition();
  tx_buf[1] = zero_pos >> 8;
  tx_buf[2] = zero_pos & 0x00FF;
  
  word max_pos = Sensor.getMaxPosition();
  tx_buf[3] = max_pos >> 8;
  tx_buf[4] = max_pos & 0x00FF;
  
  word max_ang = Sensor.getMaxAngle();
  tx_buf[5] = max_ang >> 8;
  tx_buf[6] = max_ang & 0x00FF;
  
  tx_buf[7] = Sensor.getPowerMode();
  
  tx_buf[8] = Sensor.getHysteresis();
  
  tx_buf[9] = Sensor.getOutputStage();
  
  tx_buf[10] = Sensor.getPWMFrequency();
  
  tx_buf[11] = Sensor.getSlowFilter();
  
  tx_buf[12] = Sensor.getFastFilterThreshold();
  
  tx_buf[13] = Sensor.isWatchdog();
  
  word raw_ang = Sensor.getRawAngle();
  tx_buf[14] = raw_ang >> 8;
  tx_buf[15] = raw_ang & 0x00FF;
  
  word scl_ang = Sensor.getScaledAngle();
  tx_buf[16] = scl_ang >> 8;
  tx_buf[17] = scl_ang & 0x00FF;
  
  tx_buf[18] = Sensor.getStatus();
  
  tx_buf[19] = Sensor.getAutomaticGainControl();
  
  word magn = Sensor.getMagnitude();
  tx_buf[20] = magn >> 8;
  tx_buf[21] = magn & 0x00FF;
  
  tx_buf[22] = Sensor.getBurnPositionsCount();
}
