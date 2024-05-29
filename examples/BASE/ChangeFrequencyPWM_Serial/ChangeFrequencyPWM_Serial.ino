/*
 * ChangeFrequencyPWM_Serial
 *  
 * Демонстрация изменения несущей частоты контакта OUT датчика AS5600
 * путем отправление команд через "Монитор порта"
 * 
 * Примечания:
 * 1. Если используется плата с логическими уровнями 5В, то
 *    необходимо удалить резистор R1 (0 Ом) с платы датчика,
 *    а вывод VCC подкючить к 5В питанию!
 * 
 * Подключение:
 * AS5600   Board
 * VCC   -> +3V3
 * GND   -> GND
 * DIR   -> GND
 * SDA   -> SDA
 * SCL   -> SCL
 * 
 * Проверка:
 * 1. Подключить датчик согласно распиновке
 * 2. Загрузить скетч в плату
 * 3. Открыть "Монитор порта"
 * 4. Отправлять 0, 1, 2 или 3
 * 5. Менять положение магнита
 * 4. Замерять частоту ШИМ сигнала
 *    на контакте OUT датчика
 * 
 * Документация к датчику:
 * https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
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

// Подключаем библиотеку
#include <AMS_AS5600.h>

// Создаем объект Sensor с указанием ссылки на объект Wire
AS5600 Sensor(&Wire);

void setup() {
  Serial.begin(115200);
  
  // Запускаем соединение
  Sensor.begin();
  // Настраиваем шину I2C на 400кГц
  Sensor.setClock();
  //Можно на друие частоты, но работает не на всех микроконтроллерах
  //Sensor.setClock(AS5600_I2C_CLOCK_100KHZ); // 100кГц
  //Sensor.setClock(AS5600_I2C_CLOCK_1MHZ); // 1МГц
  //Sensor.setClock(725000); // Пользовательское значение 725кГц

  // Подробно показано в примере ChangeModeOutputPin_Serial
  if(Sensor.enableOutputDigitalPWMVerify()) {
    Serial.println("Включен режим ШИМ");
    Serial.println();
  } else {
    Serial.println("Не удалось включить режим ШИМ");
    while(1);
  }
}

void loop() {
  // Проверка данных в буфере UART
  if(Serial.available()) {
    uint8_t num_mode = Serial.read(); // Чтение команды из "Монитора порта"

    // Установка разных режимов
    if(num_mode == '0') {
      Sensor.enablePWMFrequency115Hz(); // Включить ШИМ 115Гц
      Serial.println("Включен ШИМ 115Гц");
    }else if(num_mode == '1') {
      Sensor.enablePWMFrequency230Hz(); // Включить ШИМ 230Гц
      Serial.println("Включен ШИМ 230Гц");
    }else if(num_mode == '2') {
      Sensor.enablePWMFrequency460Hz(); // Включить ШИМ 460Гц
      Serial.println("Включен ШИМ 460Гц");
    }else if(num_mode == '3') {
      Sensor.enablePWMFrequency920Hz(); // Включить ШИМ 920Гц
      Serial.println("Включен ШИМ 920Гц");
    }else {
      Serial.println("Ошибка! Можно вводить только: 0, 1, 2, 3"); // Если отправленное значение режима не 0, 1, 2, 3
    }
  }
}

/*
 * Другие методы:
 * AS5600PWMFrequency getPWMFrequency(void); // Получить параметры гистерезиса
 * void getPWMFrequency(AS5600PWMFrequency &_pwm_frequency); // Тоже самое, но через ссылку
 * void setPWMFrequency(AS5600PWMFrequency _pwm_frequency); // Установить новые параметры гистерезиса
 * bool setPWMFrequencyVerify(AS5600PWMFrequency _pwm_frequency); // с подтверждением
 * 
 * _pwm_frequency:
 * AS5600_PWM_FREQUENCY_115HZ
 * AS5600_PWM_FREQUENCY_230HZ
 * AS5600_PWM_FREQUENCY_460HZ
 * AS5600_PWM_FREQUENCY_920HZ
 * 
 * с проверкой:
 * bool enablePWMFrequency115HzVerify()
 * bool enablePWMFrequency230HzVerify()
 * bool enablePWMFrequency460HzVerify()
 * bool enablePWMFrequency920HzVerify()
 */
