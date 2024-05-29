/* 
 * Класс для Arduino IDE реализующий дополнительные методы
 * для взаимодействия с бесконтактным датчиком положения
 * AS5600L от компании AMS https://ams-osram.com/
 * Этот класс основывается на классе AS5600 и является
 * частю бибилиотеки AMS-AS5600-Arduino-Library
 * 
 * Документация к датчику:
 ** https://look.ams-osram.com/m/657fca3b775890b7/original/AS5600L-DS000545.pdf
 ** https://ams-osram.com/products/sensors/position-sensors/ams-as5600l-magnetic-rotary-position-sensor
 *
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2024. v1.1 / License MIT / Скляр Роман S-LAB
 */

#include "AMS_AS5600L.h"

// ########## CONSTRUCTOR ##########
/*
 * @brief: использовать и адреса интерфейс I2C
 * @param _twi: доступ к методам объекта Wire
 * @param _iic_address: адрес датчика на шине I2C
 */
AS5600L::AS5600L(TwoWire* _twi, uint8_t _iic_address) : AS5600(_twi) {
  _i2c_address_ = _iic_address;
}
 
// ########## PROTECTED ##########
/* 
 * @brief: передать один байт адреса регистра с которого будет идти чтение
 */
void AS5600L::AS_SendFirstRegister(uint8_t _reg_addr) {
  // Начать передачу по адресу
  _wire_->beginTransmission(_i2c_address_);
  // Отправить байт регистра
  _wire_->write(_reg_addr);
  // Завершить соединение
  _wire_->endTransmission();
}
/* 
 * @brief: запросить один байт данных из буфера
 * @return: значение байта из регистра, который был запрошен ранее
 * @note: использовать для одиночного регистра, например 0x1A
 */
uint8_t AS5600L::AS_RequestSingleRegister(void) {
  uint8_t single_byte = 0;
  
  // Запросить байт данных по адресу
  _wire_->requestFrom(_i2c_address_, (uint8_t)1);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    single_byte = _wire_->read();
  }
  // Завершить соединение
  _wire_->endTransmission();

  return single_byte;
}
/* 
 * @brief: запросить два байта данных из буфера
 * @return: значения 2 байтов из регистров, которые были запрошены ранее в виде uint16_t
 * @note: использовать для парных регистров, например 0x0C 0x0D
 */
uint16_t AS5600L::AS_RequestPairRegisters(void) {
  uint8_t low_byte = 0;
  uint8_t high_byte = 0;
  
  // Запросить два байта данных по адресу
  _wire_->requestFrom(_i2c_address_, (uint8_t)2);
  // Прочитать данные из буфера
  if (_wire_->available() >= 1 ) {
    high_byte = _wire_->read();
    low_byte = _wire_->read();
  }
  // Завершить соединение
  _wire_->endTransmission();
  
  return ((high_byte << 8) | low_byte);
}
/*
 * @brief: записать значение размером 1 байт в произвольный регистр размером 1 байт
 * @param _reg: 1 байт адреса регистра
 * @param _payload: 1 байт полезных данных
 */
void AS5600L::AS_WriteOneByte(uint8_t _reg, uint8_t _payload) {
  // Начать передачу по адресу для прередачи байта данных в регистр
  _wire_->beginTransmission(_i2c_address_);
  _wire_->write(_reg);
  _wire_->write(_payload);
  // Завершить соединение
  _wire_->endTransmission();
}
/*
 * @brief: записать значение размером 2 байта в произвольный регистр размером 2 байта
 * @param _low_register: младший байт регистра
 * @param _high_register: старший байт регистра
 * @param _payload: 2 байта полезных данных
 */
void AS5600L::AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload) {
  // Начать передачу по адресу для прередачи старшего байта данных в старший регистр
  _wire_->beginTransmission(_i2c_address_);
  _wire_->write(_high_register);
  _wire_->write(_payload >> 8);
  // Завершить соединение
  _wire_->endTransmission();
  // Начать передачу по адресу для передачи младшего байта данных в младший регистр
  _wire_->beginTransmission(_i2c_address_);
  _wire_->write(_low_register);
  _wire_->write(_payload & 0xFF);
  // Завершить соединение
  _wire_->endTransmission();
}

// ########## PUBLIC ##########
/*
 * @brief: загружает данные из энергонезависимой памяти датчика в регистры ZPOS(11:0), MPOS(11:0), MANG(11:0), CONF(13:0), I2CADDR(6:0)
 *  если были установлены какие-либо значения в эти регистры то, они будут заменены значениями из энергонезависимой памяти
 * @note: назначение каждой команды не описано в документации, порядок команд описан в -
 *  Option A: Angle Programming Through the I²C Interface (Step 7)
 *  Option C: Programming a Maximum Angular Range Through the I²C Interface (Step 4)
 */
void AS5600L::loadSavedValues(void) {
  // Начать передачу по адресу
  _wire_->beginTransmission(_i2c_address_);
  _wire_->write(AS5600_BURN_REG);
  // Отправить 0x01
  _wire_->write(AS5600_CMD_BURN_LOAD_OTP_CONTENT_0);
  // Завершить соединение
  _wire_->endTransmission();
  
  // Начать передачу по адресу
  _wire_->beginTransmission(_i2c_address_);
  _wire_->write(AS5600_BURN_REG);
  // Отправить 0x11
  _wire_->write(AS5600_CMD_BURN_LOAD_OTP_CONTENT_1);
  // Завершить соединение
  _wire_->endTransmission();
  
  // Начать передачу по адресу
  _wire_->beginTransmission(_i2c_address_);
  _wire_->write(AS5600_BURN_REG);
  // Отправить 0x10
  _wire_->write(AS5600_CMD_BURN_LOAD_OTP_CONTENT_2);
  // Завершить соединение
  _wire_->endTransmission();
}
/*
 * @brief: узнать подключен ли датчик к линии I2C
 * @note: используется алгоритм стандортного поиска устройств на шина I2C
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не подключен
 *  AS5600_DEFAULT_REPORT_OK - подключен
 */
bool AS5600L::isConnected(void) {
  // Начать передачу по адресу
  _wire_->beginTransmission(_i2c_address_);
  return (!_wire_->endTransmission(_i2c_address_)) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: получить первый доступный адрес на шине I2C
 * @note: используется алгоритм стандартного поиска устройств на шина I2C
 *  применять с единственным датчиком на шине, адрес которого неизвестен
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не найден
 *  0x08 - 0x77 (8 - 119) - найден на одном из адресов
 */
byte AS5600L::findDevice(void) {
  byte error, address;
 
  for (address = RESERVED_I2C_ADDR_L + 1; address < RESERVED_I2C_ADDR_H; address++) {
    _wire_->beginTransmission(address);
    error = _wire_->endTransmission();
 
    if (error == 0) {
      return address;
    }   
  }

  return AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: получить первый доступный адрес на шине I2C
 * @note: метод работает через ссылку
 * @note: используется алгоритм стандартного поиска устройств на шина I2C
 *  применять с единственным датчиком на шине, адрес которого неизвестен
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не найден
 *  0x08 - 0x77 (8 - 119) - найден на одном из адресов
 */
void AS5600L::findDevice(byte &_i2c_address) {
  byte error, address;
 
  for (address = RESERVED_I2C_ADDR_L + 1; address < RESERVED_I2C_ADDR_H; address++) {
    _wire_->beginTransmission(address);
    error = _wire_->endTransmission();
 
    if (error == 0) {
      _i2c_address = address;
      return;
    }   
  }

  _i2c_address = AS5600_DEFAULT_REPORT_ERROR;
}
/*********************************/
/**** CONFIGURATION REGISTERS ****/
/*********************************/
/* 
 * @brief: получить значение I2C адреса из регистра I2CADDR(6:0)
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 * @return:
 *  0x08 - 0x77 (8 - 119)
 */
byte AS5600L::getRegisterAddressI2C(void) {
  AS_SendFirstRegister(AS5600L_CONFIG_REG_I2CADDR);
  return (AS_RequestSingleRegister() >> 1);
}
/* 
 * @brief: получить значение I2C адреса из регистра I2CADDR(6:0)
 * @note: метод работает через ссылку
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 * @return:
 *  0x08 - 0x77 (8 - 119)
 */
void AS5600L::getRegisterAddressI2C(byte &_i2c_address) {
  AS_SendFirstRegister(AS5600L_CONFIG_REG_I2CADDR);
  _i2c_address = (AS_RequestSingleRegister() >> 1);
}
/* 
 * @brief: установить новое значение I2C адреса в регистр I2CADDR(6:0)
 * @param _new_i2c_address: новое значение адреса I2C
 *  диапазон 0x08 - 0x77 (8 - 119)
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 */
void AS5600L::setRegisterAddressI2C(byte _new_i2c_address) {
  AS_WriteOneByte(AS5600L_CONFIG_REG_I2CADDR, (_new_i2c_address << 1));
}
/* 
 * @brief: установить новое значение I2C адреса в регистр I2CADDR(6:0) с подтверждением
 * @param _new_i2c_address: новое значение адреса I2C
 *  диапазон 0x08 - 0x77 (8 - 119)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не установлено
 *  AS5600_DEFAULT_REPORT_OK - установлено
 */
bool AS5600L::setRegisterAddressI2CVerify(byte _new_i2c_address) {
  setRegisterAddressI2C(_new_i2c_address);
  return (getRegisterAddressI2C() == _new_i2c_address) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: получить значение I2C адреса из регистра I2CUPDT I2CSTRB(6:0)
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 * @return:
 *  0x08 - 0x77 (8 - 119)
 */
byte AS5600L::getRegisterUpdateI2C(void) {
  AS_SendFirstRegister(AS5600L_CONFIG_REG_I2CUPDT);
  return (AS_RequestSingleRegister() >> 1);
}
/* 
 * @brief: получить значение I2C адреса из регистра I2CUPDT I2CSTRB(6:0)
 * @note: метод работает через ссылку
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 * @return:
 *  0x08 - 0x77 (8 - 119)
 */
void AS5600L::getRegisterUpdateI2C(byte &_i2c_address) {
  AS_SendFirstRegister(AS5600L_CONFIG_REG_I2CUPDT);
  _i2c_address = (AS_RequestSingleRegister() >> 1);
}
/* 
 * @brief: установить новое значение I2C адреса в регистр I2CUPDT I2CSTRB(6:0)
 * @param _new_i2c_address: новое значение адреса I2C
 *  диапазон 0x08 - 0x77 (8 - 119)
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 */
void AS5600L::setRegisterUpdateI2C(byte _new_i2c_address) {
  AS_WriteOneByte(AS5600L_CONFIG_REG_I2CUPDT, (_new_i2c_address << 1));
}
/* 
 * @brief: установить новое значение I2C адреса в регистр I2CUPDT I2CSTRB(6:0) с подтверждением
 * @param _new_i2c_address: новое значение адреса I2C
 *  диапазон 0x08 - 0x77 (8 - 119)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не установлено
 *  AS5600_DEFAULT_REPORT_OK - установлено
 */
bool AS5600L::setRegisterUpdateI2CVerify(byte _new_i2c_address) {
  setRegisterAddressI2C(_new_i2c_address);
  return (getRegisterAddressI2C() == _new_i2c_address) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: получить значение I2C адреса датчика
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 * @return:
 *  0x08 - 0x77 (8 - 119)
 */
byte AS5600L::getAddressI2C(void) {
  _i2c_address_ = getRegisterUpdateI2C();
  return _i2c_address_;
}
/* 
 * @brief: получить значение I2C адреса датчика
 * @note: метод работает через ссылку
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 * @return:
 *  0x08 - 0x77 (8 - 119)
 */
void AS5600L::getAddressI2C(byte &_i2c_address) {
  _i2c_address_ = getRegisterUpdateI2C();
  _i2c_address = _i2c_address_;
}
/* 
 * @brief: установить новое значение I2C адреса датчика
 * @param _new_i2c_address: новое значение адреса I2C
 *  диапазон 0x08 - 0x77 (8 - 119)
 * @note: адреса вне диапазона зарезервированы протоколом I2C
 */
void AS5600L::setAddressI2C(byte _new_i2c_address) {
  if ((_new_i2c_address > RESERVED_I2C_ADDR_L) && (_new_i2c_address < RESERVED_I2C_ADDR_H)) {
    _i2c_address_ = _new_i2c_address;
    setRegisterAddressI2C(_new_i2c_address);
    setRegisterUpdateI2C(_new_i2c_address);
  }
}
/* 
 * @brief: установить новое значение I2C адреса датчика с подтверждением
 * @param _new_i2c_address: новое значение адреса I2C
 *  диапазон 0x08 - 0x77 (8 - 119)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не установлено
 *  AS5600_DEFAULT_REPORT_OK - установлено
 */
bool AS5600L::setAddressI2CVerify(byte _new_i2c_address) {
  setAddressI2C(_new_i2c_address);
  return (getAddressI2C() == _new_i2c_address) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: записать НАВСЕГДА установленные значения в регистрах MANG(11:0), CONF(13:0), I2CADDR(6:0)
 * @note: ВЫПОЛНИТЬ ЭТУ КОМАНДУ МОЖНО ТОЛЬКО 1(ОДИН) РАЗ ДЛЯ ОДНОГО ДАТЧИКА 
 *  ПРИ НАЛИЧИИ МАГНИТА (MD:5 = 1) И ПРИ НАЛИЧИИ РЕСУРСА В ZMCO(1:0)!
 * @param _use_special_verify:
 *  AS5600_FLAG_SPECIAL_VERIFY_DISABLE
 *  AS5600_FLAG_SPECIAL_VERIFY_ENABLE
 * @return:
 *  AS5600_BURN_REPROT_SENSOR_NOT_CONNECTED
 *  AS5600_BURN_REPROT_MAGNET_NOT_FOUND
 *  AS5600_BURN_REPROT_WRITE_OK
 *  AS5600_BURN_REPROT_WRITE_WRONG
 *  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY
 *  AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL
 *  AS5600_BURN_REPROT_ATTEMPTS_ENDED
 */
AS5600BurnReports AS5600L::burnMaxAngleAndConfigurationValue(AS5600SpecialVerifyFlags _use_special_verify) {
  AS5600BurnReports result = AS5600_BURN_REPROT_SENSOR_NOT_CONNECTED;
  
  if (isConnected()) { // Если датчик подключен
    // Собираем значениях из критически выжных регистров
    byte burn_count = getBurnPositionsCount();
    word m_ang = getMaxAngle();
    word conf = getRawConfigurationValue();
    byte addr = getRegisterAddressI2C();
    if (burn_count == 0) { // Если ресурс для записи не исчерпан
      if (getMaxAngle() >= AS5600_MIN_ANGLE_VALUE_DEC) { // Если значение угла подходит
        // Наличие магнита проверяем НА ПОСЛЕДНЕМ ШАГЕ, перед отправлением команды на запись!
        if (isMagnetDetected()) { // Если магнит обнаружен
          AS_WriteOneByte(AS5600_BURN_REG, AS5600_CMD_BURN_SETTINGS); // Отправляем команду записи настроек
          if (_use_special_verify) { // Если используется проверка записанного
            loadSavedValues(); // Загружаем из памяти ранее записанные данные
            // Получаем загруженные данные для сравнения
            word m_ang_now = getMaxAngle();
            word conf_now = getRawConfigurationValue();
            byte addr_now = getRegisterAddressI2C();
            if (m_ang == m_ang_now && conf == conf_now && addr == addr_now) { // Если записываемые данные совпадают с сохраненными
              result = AS5600_BURN_REPROT_WRITE_OK;
            } else {
              result = AS5600_BURN_REPROT_WRITE_WRONG;
            }
          } else {
            result = AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY;
          }
        } else {
          result = AS5600_BURN_REPROT_MAGNET_NOT_FOUND;
        }
      } else {
        result = AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL;
      }
    } else {
      result = AS5600_BURN_REPROT_ATTEMPTS_ENDED;
    }
  }

  return result;
}
