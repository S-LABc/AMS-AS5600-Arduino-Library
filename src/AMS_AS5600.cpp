/* 
 * Класс для Arduino IDE реализующий множество методов
 * взаимодействия с бесконтактным датчиком положения
 * AS5600 от компании AMS https://ams.com/ams-start
 * 
 * Документация к датчику:
 ** https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 ** https://ams.com/en/as5600
 *
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.3 / License MIT / Скляр Роман S-LAB
 */

#include "AMS_AS5600.h"

// ########## CONSTRUCTOR ##########
/*
 * @brief: использовать только интерфейс I2C
 * @param *twi: доступ к методам объекта Wire
 */
AS5600::AS5600(TwoWire *twi) : __wire(twi ? twi : &Wire) {
  // Ничего
}
/*
 * @brief: использовать интерфейс I2C, вывод МК для контакта DIR датчика, задать положительно направление вращения
 * @param *twi: доступ к методам объекта Wire
 * @para pin_dir: контакт микроконтроллера к которому подключен контакт DIR датчика
 * @param def_polar_dir: положительно направление вращения магнита (по/против часовой стрелки)
 */
AS5600::AS5600(TwoWire *twi, int8_t _pin_dir, AS5600DirectionPolarity _def_polar_dir) : __wire(twi ? twi : &Wire) {
  _pin_direction_ = _pin_dir;
  pinMode(_pin_direction_, OUTPUT);
  digitalWrite(_pin_direction_, _def_polar_dir);
}
 
// ########## PROTECTED ##########
/* 
 * @brief: передать один байт адреса регистра с которого будет идти чтение
 */
void AS5600::AS_SendFirstRegister(uint8_t _reg_addr) {
  // Начать передачу по адресу 0x36
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  // Отправить байт регистра
  __wire->write(_reg_addr);
  // Завершить соединение
  __wire->endTransmission();
}
/* 
 * @brief: запросить один байт данных из буфера
 * @return: значение байта из регистра, который был запрошен ранее
 * @note: использовать для одиночного регистра, например 0x1A
 */
uint8_t AS5600::AS_RequestSingleRegister(void) {
  uint8_t single_byte = 0;
  
  // Запросить байт данных по адресу 0x36
  __wire->requestFrom(AS5600_I2C_ADDRESS, 1);
  // Прочитать данные из буфера
  if (__wire->available() >= 1 ) {
    single_byte = __wire->read();
  }
  // Завершить соединение
  __wire->endTransmission();

  return single_byte;
}
/* 
 * @brief: запросить два байта данных из буфера
 * @return: значения 2 байтов из регистров, которые были запрошены ранее в виде uint16_t
 * @note: использовать для парных регистров, например 0x0C 0x0D
 */
uint16_t AS5600::AS_RequestPairRegisters(void) {
  uint8_t low_byte = 0;
  uint8_t high_byte = 0;
  
  // Запросить два байта данных по адресу 0x36
  __wire->requestFrom(AS5600_I2C_ADDRESS, 2);
  // Прочитать данные из буфера
  if (__wire->available() >= 1 ) {
    high_byte = __wire->read();
    low_byte = __wire->read();
  }
  // Завершить соединение
  __wire->endTransmission();
  
  return ((high_byte << 8) | low_byte);
}
/*
 * @brief: записать значение размером 1 байт в произвольный регистр размером 1 байт
 * @param _reg: 1 байт адреса регистра
 * @param _payload: 1 байт полезных данных
 */
void AS5600::AS_WriteOneByte(uint8_t _reg, uint8_t _payload) {
  // Начать передачу по адресу 0x36 для прередачи байта данных в регистр
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  __wire->write(_reg);
  __wire->write(_payload);
  // Завершить соединение
  __wire->endTransmission();
}
/*
 * @brief: записать значение размером 2 байта в произвольный регистр размером 2 байта
 * @param _low_register: младший байт регистра
 * @param _high_register: старший байт регистра
 * @param _payload: 2 байта полезных данных
 */
void AS5600::AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload) {
  // Начать передачу по адресу 0x36 для прередачи старшего байта данных в старший регистр
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  __wire->write(_high_register);
  __wire->write(_payload >> 8);
  // Завершить соединение
  __wire->endTransmission();
  // Начать передачу по адресу 0x36 для передачи младшего байта данных в младший регистр
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  __wire->write(_low_register);
  __wire->write(_payload & 0xFF);
  // Завершить соединение
  __wire->endTransmission();
}

// ########## PUBLIC ##########
/* 
 * @brief: вызов метода Wire.begin()
 * @note: использовать, если действие не было выполнено ранее
 */
void AS5600::begin(void) {
  __wire->begin();
}
/* 
 * @brief: настройка частоты шины I2C
 * @note: использовать, если частота шины меняется из-за разных устройств. по умолчанию 400кГц
 */
void AS5600::setClock(uint32_t freq_hz) {
  __wire->setClock(freq_hz);
}
/* 
 * @brief: отключение шины I2C
 */
void AS5600::end(void) {
  __wire->end();
}
/*
 * @brief: загружает данные из энергонезависимой памяти датчика в регистры ZPOS(11:0), MPOS(11:0), MANG(11:0), CONF(13:0)
 *  если были установлены какие-либо значения в эти регистры то, они будут заменены значениями из энергонезависимой памяти
 * @note: назначение каждой команды не описано в документации, порядок команд описан в -
 *  Option A: Angle Programming Through the I²C Interface (Step 7)
 *  Option C: Programming a Maximum Angular Range Through the I²C Interface (Step 4)
 */
void AS5600::loadSavedValues(void) {
  // Начать передачу по адресу 0x36
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  __wire->write(AS5600_BURN_REG);
  // Отправить 0x01
  __wire->write(AS5600_CMD_BURN_LOAD_OTP_CONTENT_0);
  // Завершить соединение
  __wire->endTransmission();
  
  // Начать передачу по адресу 0x36
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  __wire->write(AS5600_BURN_REG);
  // Отправить 0x11
  __wire->write(AS5600_CMD_BURN_LOAD_OTP_CONTENT_1);
  // Завершить соединение
  __wire->endTransmission();
  
  // Начать передачу по адресу 0x36
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  __wire->write(AS5600_BURN_REG);
  // Отправить 0x10
  __wire->write(AS5600_CMD_BURN_LOAD_OTP_CONTENT_2);
  // Завершить соединение
  __wire->endTransmission();
}
/*
 * @brief: узнать подкючен ли датчик к линии I2C
 * @note: используется алгоритм стандортного поиска устройств на шина I2C
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не подключен
 *  AS5600_DEFAULT_REPORT_OK - подключен
 */
bool AS5600::isConnected(void) {
  // Начать передачу по адресу 0x36
  __wire->beginTransmission(AS5600_I2C_ADDRESS);
  return (!__wire->endTransmission(AS5600_I2C_ADDRESS)) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}

/*
 * @brief: установить новое минимальное значение срабатывания кнопки
 * @param _btn_min_agc: нижняя граница срабатывания кнопки
 */
void AS5600::setButtonMinAGC(byte _btn_min_agc) {
  _virtual_button.minimum_agc = _btn_min_agc;
}
/*
 * @brief: получить минимальное значение срабатывания кнопки
 */
byte AS5600::getButtonMinAGC(void) {
  return _virtual_button.minimum_agc;
}
/*
 * @brief: установить новое максимальное значение срабатывания кнопки
 * @param _btn_max_agc: верхняя граница срабатывания кнопки
 */
void AS5600::setButtonMaxAGC(byte _btn_max_agc) {
  _virtual_button.maximum_agc = _btn_max_agc;
}
/*
 * @brief: получить максимальное значение срабатывания кнопки
 */
byte AS5600::getButtonMaxAGC(void) {
  return _virtual_button.maximum_agc;
}
/*
 * @brief: установить новое значение отклонения срабатывания кнопки
 * @param _btn_div: значение отклонения
 */
void AS5600::setButtonDeviation(byte _btn_div) {
  _virtual_button.deviation = _btn_div;
}
/*
 * @brief: получить значение отклонения срабатывания кнопки
 */
byte AS5600::getButtonDeviation(void) {
  return _virtual_button.deviation;
}
/*
 * @brief: узнать нажата ли виртуальная кнопка
 * @note: метод построен на обработке значения от метода getAutomaticGainControl
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - кнопка не нажата
 *  AS5600_DEFAULT_REPORT_OK - кнопка нажата
 */
bool AS5600::isButtonPressed(void) {
  byte agc_value = AS5600::getAutomaticGainControl();
  if(!_virtual_button.falg_button_state && (agc_value < (_virtual_button.minimum_agc + _virtual_button.deviation))) {
    _virtual_button.falg_button_state = true;
	return AS5600_DEFAULT_REPORT_OK;
  }else {
	return AS5600_DEFAULT_REPORT_ERROR;
  }
}
/*
 * @brief: узнать отпущена ли виртуальная кнопка
 * @note: метод построен на обработке значения от метода getAutomaticGainControl
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - кнопка не отпущена
 *  AS5600_DEFAULT_REPORT_OK - кнопка отпущена
 */
bool AS5600::isButtonReleased(void) {
  byte agc_value = AS5600::getAutomaticGainControl();
  if(_virtual_button.falg_button_state && (agc_value > (_virtual_button.maximum_agc - _virtual_button.deviation))) {
    _virtual_button.falg_button_state = false;
	return AS5600_DEFAULT_REPORT_OK;
  }else {
	return AS5600_DEFAULT_REPORT_ERROR;
  }
}
/*
 * @brief: назначить контакт микроконтроллера для управления контактом DIR датчика
 * @param _pin_dir: контакт микроконтроллера к которому подключен контакт DIR датчика
 */
void AS5600::attachDirectionPin(byte _pin_dir) {
  _pin_direction_ = _pin_dir;
  pinMode(_pin_direction_, OUTPUT);
}
/* 
 * @brief: установить положительное направление вращения по часовой стрелке или против часовой стрелки
 * @param _direction_polarity: положительное направление вращения
 *  AS5600_DIRECTION_POLARITY_CLOCKWISE - по часовой стрелке
 *  AS5600_DIRECTION_POLARITY_COUNTERCLOCKWISE - против часовй стрелки
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - контакт МК не назначен
 *  AS5600_DEFAULT_REPORT_OK - направление изменено
 */
bool AS5600::setDirection(AS5600DirectionPolarity _direction_polarity) {
  if(_pin_direction_ == -1) {
    return AS5600_DEFAULT_REPORT_ERROR;
  }
  digitalWrite(_pin_direction_, _direction_polarity);
  
  return AS5600_DEFAULT_REPORT_OK;
}
/* 
 * @brief: изменить положительное направление вращения на противоположное
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - контакт МК не назначен
 *  AS5600_DEFAULT_REPORT_OK - направление изменено
 */
bool AS5600::reverseDirection(void) {
  if(_pin_direction_ == -1) {
    return AS5600_DEFAULT_REPORT_ERROR;
  }
  digitalWrite(_pin_direction_, !digitalRead(_pin_direction_));
  
  return AS5600_DEFAULT_REPORT_OK;
}
/*
 * @brief: получить текущее положительное направление вращения
 * @return:
 *  AS5600_DIRECTION_POLARITY_CLOCKWISE - направление по часовой стрелке
 *  AS5600_DIRECTION_POLARITY_COUNTERCLOCKWISE - направление против часовой стрелки
 */
bool AS5600::getDirection(void) {
  return (bool)digitalRead(_pin_direction_);
}
/*********************************/
/**** CONFIGURATION REGISTERS ****/
/*********************************/
/*
 * @brief: получить количество записей значений в ZPOS и MPOS через BURN_ANGLE 0x80 в регистр BURN
 * @note: ZMCO(1:0)
 * @return:
 *  0 - заводское значение
 *  1 - в ZPOS записано один раз
 *  2 - в ZPOS записано два раза
 *  3 - в ZPOS записано три раза
 */
byte AS5600::getBurnPositionsCount(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_ZMCO);
  return AS5600::AS_RequestSingleRegister();
}
/* 
 * @brief: получить значение начального положения из регистра ZPOS(11:0) (начальный угол)
 * @return:
 *  0 - 4095
 */
word AS5600::getZeroPosition(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_ZPOS_H);
  return AS5600::AS_RequestPairRegisters();
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0)
 * @param _zero_position:
 *  0 - 4095
 */
void AS5600::setZeroPosition(word _zero_position) {
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_ZPOS_L, AS5600_CONFIG_REG_ZPOS_H, _zero_position);
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) с подтверждением
 * @param _zero_position:
 *  0 - 4095
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK- новое значение успешно установлено
 */
bool AS5600::setZeroPositionVerify(word _zero_position) {
  AS5600::setZeroPosition(_zero_position);
  return (AS5600::getZeroPosition() == _zero_position) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) используя нынешнее положение магнита
 * @note: получает и отправляет значение полученное от метода getRawAngle
 */
void AS5600::setZeroPositionViaRawAngle(void) {
  word raw_angle = AS5600::getRawAngle();
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_ZPOS_L, AS5600_CONFIG_REG_ZPOS_H, raw_angle);
}
/* 
 * @brief: установить новое начальное положение в регистр ZPOS(11:0) используя нынешнее положение магнита с подтверждением
 * @note: получает и отправляет значение полученное от метода getRawAngle
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK- новое значение успешно установлено
 */
bool AS5600::setZeroPositionViaRawAngleVerify(void) {
  word raw_angle = AS5600::getRawAngle();
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_ZPOS_L, AS5600_CONFIG_REG_ZPOS_H, raw_angle);
  return (AS5600::getZeroPosition() == raw_angle) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * brief: получить значение конечного положения из регистра MPOS(11:0) (конечный угол)
 * @return:
 *  0 - 4095
 */
word AS5600::getMaxPosition(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_MPOS_H);
  return AS5600::AS_RequestPairRegisters();
}
/* 
 * @brief: установить новое конечное положение в регистр MPOS(11:0)
 * @param _max_position:
 *  0 - 4095
 */
void AS5600::setMaxPosition(word _max_position) {
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_MPOS_L, AS5600_CONFIG_REG_MPOS_H, _max_position);
}
/* 
 * @brief: установить новое конечное положение в регистр MPOS(11:0) с подтверждением
 * @param _max_position:
 *  0 - 4095
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK - новое значение успешно установлено
 */
bool AS5600::setMaxPositionVerify(word _max_position) {
  AS5600::setMaxPosition(_max_position);
  return (AS5600::getMaxPosition() == _max_position) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: установить новое конечное положение в регистр MPOS(11:0) используя нынешнее положение магнита
 * @note: получает и отправляет значение полученное от метода getRawAngle
 */
void AS5600::setMaxPositionViaRawAngle(void) {
  word raw_angle = AS5600::getRawAngle();
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_MPOS_L, AS5600_CONFIG_REG_MPOS_H, raw_angle);
}
/* 
 * @brief: установить новое конечное положение в регистр MPOS(11:0) используя нынешнее положение магнита с подтверждением
 * @note: получает и отправляет значение полученное от метода getRawAngle
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK- новое значение успешно установлено
 */
bool AS5600::setMaxPositionViaRawAngleVerify(void) {
  word raw_angle = AS5600::getRawAngle();
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_MPOS_L, AS5600_CONFIG_REG_MPOS_H, raw_angle);
  return (AS5600::getMaxPosition() == raw_angle) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: получить значение максимального угла из регистра MANG(11:0)
 * @return:
 *  0 - 4095
 */
word AS5600::getMaxAngle(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_MANG_H);
  return AS5600::AS_RequestPairRegisters();
}
/* 
 * @brief: установить новое значение максимального угла в регистр MANG(11:0)
 * @param _max_angle:
 *  0 - 4095
 */
void AS5600::setMaxAngle(word _max_angle) {
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_MANG_L, AS5600_CONFIG_REG_MANG_H, _max_angle);
}
/* 
 * @brief: установить новое значение максимального угла в регистр MANG(11:0) с подтверждением
 * @param _max_angle:
 *  0 - 4095
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK - новое значение успешно установлено
 */
bool AS5600::setMaxAngleVerify(word _max_angle) {
  AS5600::setMaxAngle(_max_angle);
  return (AS5600::getMaxAngle() == _max_angle) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: установить новое значение максимального угла в регистр MANG(11:0) используя нынешнее положение магнита
 * @note: получает и отправляет значение полученное от метода getRawAngle
 */
void AS5600::setMaxAngleViaRawAngle(void) {
  word raw_angle = AS5600::getRawAngle();
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_MANG_L, AS5600_CONFIG_REG_MANG_H, raw_angle);
}
/* 
 * @brief: установить новое значение максимального угла в регистр MANG(11:0) используя нынешнее положение магнита с подтверждением
 * @note: получает и отправляет значение полученное от метода getRawAngle
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK- новое значение успешно установлено
 */
bool AS5600::setMaxAngleViaRawAngleVerify(void) {
  word raw_angle = AS5600::getRawAngle();
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_MANG_L, AS5600_CONFIG_REG_MANG_H, raw_angle);
  return (AS5600::getMaxAngle() == raw_angle) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/* 
 * @brief: получить значение конфигураций из регистра CONF(13:0)
 * @return: целое шестнадцатиричное число
 */
word AS5600::getRawConfigurationValue(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  return AS5600::AS_RequestPairRegisters();
}
/* 
 * @brief: установить новое значение конфигураций в регистр CONF(13:0)
 * @param _confuration_value: целое шестнадцатиричное число
 */
void AS5600::setRawConfigurationValue(word _confuration_value) {
  AS5600::AS_WriteTwoBytes(AS5600_CONFIG_REG_CONF_L, AS5600_CONFIG_REG_CONF_H, _confuration_value);
}
/* 
 * @brief: установить новое значение конфигураций в регистр CONF(13:0) с подтверждением
 * @param _confuration_value: новое значение конфигураций
 *  AS5600_DEFAULT_REPORT_ERROR - новое значение не установлено
 *  AS5600_DEFAULT_REPORT_OK - новое значение успешно установлено
 */
bool AS5600::setRawConfigurationValueVerify(word _confuration_value) {
  AS5600::setRawConfigurationValue(_confuration_value);
  return (AS5600::getRawConfigurationValue() == _confuration_value) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: получить значение текущего режима питания. биты (PM:0,PM:1) регистра CONF(1:0)
 * @return: 
 *  AS5600_NOM_POWER_MODE
 *  AS5600_LOW_POWER_MODE_1
 *  AS5600_LOW_POWER_MODE_2
 *  AS5600_LOW_POWER_MODE_3
 */
AS5600PowerModes AS5600::getPowerMode(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  return (AS5600PowerModes)(AS5600::AS_RequestSingleRegister() & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новое значение режима питания. биты (PM:0,PM:1) регистра CONF(1:0)
 * @param _power_mode:
 *  AS5600_NOM_POWER_MODE
 *  AS5600_LOW_POWER_MODE_1
 *  AS5600_LOW_POWER_MODE_2
 *  AS5600_LOW_POWER_MODE_3
 */
void AS5600::setPowerMode(AS5600PowerModes _power_mode) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  uint8_t conf_l_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_L, conf_l_raw |= _power_mode);
}
/*
 * @brief: установить новое значение режима питания с подтверждением. биты (PM:0,PM:1) регистра CONF(1:0)
 * @param _power_mode:
 *  AS5600_NOM_POWER_MODE
 *  AS5600_LOW_POWER_MODE_1
 *  AS5600_LOW_POWER_MODE_2
 *  AS5600_LOW_POWER_MODE_3
 * @return: 
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить новый режим 
 *  AS5600_DEFAULT_REPORT_OK - новый режим установлен
 */
bool AS5600::setPowerModeVerify(AS5600PowerModes _power_mode) {
  AS5600::setPowerMode(_power_mode);
  return (AS5600::getPowerMode() == _power_mode) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить нормальный режим питания. биты (PM:0,PM:1) регистра CONF(1:0)
 */
void AS5600::enableNomPowerMode(void) {
  AS5600::setPowerMode(AS5600_NOM_POWER_MODE);
}
/*
 * @brief: включить нормальный режим питания с подтверждением. биты (PM:0,PM:1) регистра CONF(1:0)
 * @return: 
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5600_DEFAULT_REPORT_OK - режим включиен
 */
bool AS5600::enableNomPowerModeVerify(void) {
  return AS5600::setPowerModeVerify(AS5600_NOM_POWER_MODE);
}
/*
 * @brief: включить режим питания 1. биты (PM:0,PM:1) регистра CONF(1:0)
 */
void AS5600::enableLowPowerMode1(void) {
  AS5600::setPowerMode(AS5600_LOW_POWER_MODE_1);
}
/*
 * @brief: включить режим питания 1 с подтверждением. биты (PM:0,PM:1) регистра CONF(1:0)
 * @return: 
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5600_DEFAULT_REPORT_OK - режим включиен
 */
bool AS5600::enableLowPowerMode1Verify(void) {
  return AS5600::setPowerModeVerify(AS5600_LOW_POWER_MODE_1);
}
/*
 * @brief: включить режим питания 2. биты (PM:0,PM:1) регистра CONF(1:0)
 */
void AS5600::enableLowPowerMode2(void) {
  AS5600::setPowerMode(AS5600_LOW_POWER_MODE_2);
}
/*
 * @brief: включить режим питания 2 с подтверждением. биты (PM:0,PM:1) регистра CONF(1:0)
 * @return: 
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5600_DEFAULT_REPORT_OK - режим включиен
 */
bool AS5600::enableLowPowerMode2Verify(void) {
  return AS5600::setPowerModeVerify(AS5600_LOW_POWER_MODE_2);
}
/*
 * @brief: включить режим питания 3. биты (PM:0,PM:1) регистра CONF(1:0)
 */
void AS5600::enableLowPowerMode3(void) {
  AS5600::setPowerMode(AS5600_LOW_POWER_MODE_3);
}
/*
 * @brief: включить режим питания 3 с подтверждением. биты (PM:0,PM:1) регистра CONF(1:0)
 * @return: 
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить режим 
 *  AS5600_DEFAULT_REPORT_OK - режим включиен
 */
bool AS5600::enableLowPowerMode3Verify(void) {
  return AS5600::setPowerModeVerify(AS5600_LOW_POWER_MODE_3);
}
/*
 * @brief: получить установленное значение гистерезиса. биты (HYST:0,HYST:1) регистра CONF(1:0)
 * @return:
 *  AS5600_HYSTERESIS_OFF
 *  AS5600_HYSTERESIS_1_LSB
 *  AS5600_HYSTERESIS_2_LSB
 *  AS5600_HYSTERESIS_3_LSB
 */
AS5600Hysteresis AS5600::getHysteresis(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  return (AS5600Hysteresis)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_HYST_0) & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новые значения гистерезиса. биты (HYST:0,HYST:1) регистра CONF(3:2)
 * @param _hysteresis:
 *  AS5600_HYSTERESIS_OFF
 *  AS5600_HYSTERESIS_1_LSB
 *  AS5600_HYSTERESIS_2_LSB
 *  AS5600_HYSTERESIS_3_LSB
 */
void AS5600::setHysteresis(AS5600Hysteresis _hysteresis) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  uint8_t conf_l_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_L, conf_l_raw |= (_hysteresis << AS5600_CONF_BIT_HYST_0));
}
/*
 * @brief: установить новые значения гистерезиса с подтверждением. биты (HYST:0,HYST:1) регистра CONF(3:2)
 * @param _hysteresis:
 *  AS5600_HYSTERESIS_OFF
 *  AS5600_HYSTERESIS_1_LSB
 *  AS5600_HYSTERESIS_2_LSB
 *  AS5600_HYSTERESIS_3_LSB
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить новое значение
 *  AS5600_DEFAULT_REPORT_OK - новое значение установлено
 */
bool AS5600::setHysteresisVerify(AS5600Hysteresis _hysteresis) {
  AS5600::setHysteresis(_hysteresis);
  return (AS5600::getHysteresis() == _hysteresis) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: выключить гистерезис (HYST-00)
 */
void AS5600::disableHysteresis(void) {
  AS5600::setHysteresis(AS5600_HYSTERESIS_OFF);
}
/*
 * @brief: выключить гистерезис (HYST-00) с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось выключить
 *  AS5600_DEFAULT_REPORT_OK - удалось выключить
 */
bool AS5600::disableHysteresisVerify(void) {
  return AS5600::setHysteresisVerify(AS5600_HYSTERESIS_OFF);
}
/*
 * @brief: включить гистерезис на 1 LSB (HYST-01)
 */
void AS5600::enableHysteresis1LSB(void) {
  AS5600::setHysteresis(AS5600_HYSTERESIS_1_LSB);
}
/*
 * @brief: включить гистерезис на 1 LSB (HYST-01) с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5600_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5600::enableHysteresis1LSBVerify(void) {
  return AS5600::setHysteresisVerify(AS5600_HYSTERESIS_1_LSB);
}
/*
 * @brief: включить гистерезис на 2 LSB (HYST-10)
 */
void AS5600::enableHysteresis2LSB(void) {
  AS5600::setHysteresis(AS5600_HYSTERESIS_2_LSB);
}
/*
 * @brief: включить гистерезис на 2 LSB (HYST-10) с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5600_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5600::enableHysteresis2LSBVerify(void) {
  return AS5600::setHysteresisVerify(AS5600_HYSTERESIS_2_LSB);
}
/*
 * @brief: включить гистерезис на 3 LSB (HYST-11)
 */
void AS5600::enableHysteresis3LSB(void) {
  AS5600::setHysteresis(AS5600_HYSTERESIS_3_LSB);
}
/*
 * @brief: включить гистерезис на 3 LSB (HYST-11) с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5600_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5600::enableHysteresis3LSBVerify(void) {
  return AS5600::setHysteresisVerify(AS5600_HYSTERESIS_3_LSB);
}
/*
 * @brief: получить режим работы контакта OUT
 * @return:
 *  AS5600_OUTPUT_ANALOG_FULL_RANGE
 *  AS5600_OUTPUT_ANALOG_REDUCED_RANGE
 *  AS5600_OUTPUT_DIGITAL_PWM
 */
AS5600OutputStage AS5600::getOutputStage(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  return (AS5600OutputStage)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_OUTS_0) & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить режим работы контакта OUT
 * @param _output_stage:
 *  AS5600_OUTPUT_ANALOG_FULL_RANGE
 *  AS5600_OUTPUT_ANALOG_REDUCED_RANGE
 *  AS5600_OUTPUT_DIGITAL_PWM
 */
void AS5600::setOutputStage(AS5600OutputStage _output_stage) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  uint8_t conf_l_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_L, conf_l_raw |= (_output_stage << AS5600_CONF_BIT_OUTS_0));
}
/*
 * @brief: установить режим работы контакта OUT с подтверждением
 * @param _output_stage:
 *  AS5600_OUTPUT_ANALOG_FULL_RANGE
 *  AS5600_OUTPUT_ANALOG_REDUCED_RANGE
 *  AS5600_OUTPUT_DIGITAL_PWM
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось включить
 *  AS5600_DEFAULT_REPORT_OK - удалось включить
 */
bool AS5600::setOutputStageVerify(AS5600OutputStage _output_stage) {
  AS5600::setOutputStage(_output_stage);
  return (AS5600::getOutputStage() == _output_stage) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: установить режим работы контакта OUT как аналоговый выход (0-100%)
 */
void AS5600::enableOutputAnalogFullRange(void) {
  AS5600::setOutputStage(AS5600_OUTPUT_ANALOG_FULL_RANGE);
}
/*
 * @brief: установить режим работы контакта OUT как аналоговый выход (0-100%) с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableOutputAnalogFullRangeVerify(void) {
  return AS5600::setOutputStageVerify(AS5600_OUTPUT_ANALOG_FULL_RANGE);
}
/*
 * @brief: установить режим работы контакта OUT как аналоговый выход (10-90%)
 */
void AS5600::enableOutputAnalogReducedRange(void) {
  AS5600::setOutputStage(AS5600_OUTPUT_ANALOG_REDUCED_RANGE);
}
/*
 * @brief: установить режим работы контакта OUT как аналоговый выход (10-90%) с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableOutputAnalogReducedRangeVerify(void) {
  return AS5600::setOutputStageVerify(AS5600_OUTPUT_ANALOG_REDUCED_RANGE);
}
/*
 * @brief: установить режим работы контакта OUT как цифровой ШИМ выход
 */
void AS5600::enableOutputDigitalPWM(void) {
  AS5600::setOutputStage(AS5600_OUTPUT_DIGITAL_PWM);
}
/*
 * @brief: установить режим работы контакта OUT как цифровой ШИМ выход с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableOutputDigitalPWMVerify(void) {
  return AS5600::setOutputStageVerify(AS5600_OUTPUT_DIGITAL_PWM);
}
/*
 * @brief: получить чатоту ШИМ
 * @return:
 *  AS5600_PWM_FREQUENCY_115HZ
 *  AS5600_PWM_FREQUENCY_230HZ
 *  AS5600_PWM_FREQUENCY_460HZ
 *  AS5600_PWM_FREQUENCY_920HZ
 */
AS5600PWMFrequency AS5600::getPWMFrequency(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  return (AS5600PWMFrequency)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_PWMF_0) & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новое значение частоты ШИМ
 * @param _pwm_frequency:
 *  AS5600_PWM_FREQUENCY_115HZ
 *  AS5600_PWM_FREQUENCY_230HZ
 *  AS5600_PWM_FREQUENCY_460HZ
 *  AS5600_PWM_FREQUENCY_920HZ
 */
void AS5600::setPWMFrequency(AS5600PWMFrequency _pwm_frequency) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_L);
  uint8_t conf_l_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_L, conf_l_raw |= (_pwm_frequency << AS5600_CONF_BIT_PWMF_0));
}
/*
 * @brief: установить новое значение частоты ШИМ с подтверждением
 * @param _pwm_frequency:
 *  AS5600_PWM_FREQUENCY_115HZ
 *  AS5600_PWM_FREQUENCY_230HZ
 *  AS5600_PWM_FREQUENCY_460HZ
 *  AS5600_PWM_FREQUENCY_920HZ
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::setPWMFrequencyVerify(AS5600PWMFrequency _pwm_frequency) {
  AS5600::setPWMFrequency(_pwm_frequency);
  return (AS5600::getPWMFrequency() == _pwm_frequency) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить ШИМ 115Гц
 */
void AS5600::enablePWMFrequency115Hz(void) {
  AS5600::setPWMFrequency(AS5600_PWM_FREQUENCY_115HZ);
}
/*
 * @brief: включить ШИМ 115Гц с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enablePWMFrequency115HzVerify(void) {
  return AS5600::setPWMFrequencyVerify(AS5600_PWM_FREQUENCY_115HZ);
}
/*
 * @brief: включить ШИМ 230Гц
 */
void AS5600::enablePWMFrequency230Hz(void) {
  AS5600::setPWMFrequency(AS5600_PWM_FREQUENCY_230HZ);
}
/*
 * @brief: включить ШИМ 230Гц с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enablePWMFrequency230HzVerify(void) {
  return AS5600::setPWMFrequencyVerify(AS5600_PWM_FREQUENCY_230HZ);
}
/*
 * @brief: включить ШИМ 460Гц
 */
void AS5600::enablePWMFrequency460Hz(void) {
  AS5600::setPWMFrequency(AS5600_PWM_FREQUENCY_460HZ);
}
/*
 * @brief: включить ШИМ 460Гц с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enablePWMFrequency460HzVerify(void) {
  return AS5600::setPWMFrequencyVerify(AS5600_PWM_FREQUENCY_460HZ);
}
/*
 * @brief: включить ШИМ 920Гц
 */
void AS5600::enablePWMFrequency920Hz(void) {
  AS5600::setPWMFrequency(AS5600_PWM_FREQUENCY_920HZ);
}
/*
 * @brief: включить ШИМ 920Гц с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enablePWMFrequency920HzVerify(void) {
  return AS5600::setPWMFrequencyVerify(AS5600_PWM_FREQUENCY_920HZ);
}
/*
 * @brief: получить значение коэффициента медленной фильтрации
 * @return:
 *  AS5600_SLOW_FILTER_16X
 *  AS5600_SLOW_FILTER_8X
 *  AS5600_SLOW_FILTER_4X
 *  AS5600_SLOW_FILTER_2X
 */
AS5600SlowFilter AS5600::getSlowFilter(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  return (AS5600SlowFilter)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_SF_0) & 0x03); // 0x03=0b00000011
}
/*
 * @brief: установить новое значение коэффициента медленной фильтрации
 * @param _slow_filter:
 *  AS5600_SLOW_FILTER_16X
 *  AS5600_SLOW_FILTER_8X
 *  AS5600_SLOW_FILTER_4X
 *  AS5600_SLOW_FILTER_2X
 */
void AS5600::setSlowFilter(AS5600SlowFilter _slow_filter) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_H, conf_h_raw |= (_slow_filter << AS5600_CONF_BIT_SF_0));
}
/*
 * @brief: установить новое значение коэффициента медленной фильтрации с подтверждением
 * @param _slow_filter:
 *  AS5600_SLOW_FILTER_16X
 *  AS5600_SLOW_FILTER_8X
 *  AS5600_SLOW_FILTER_4X
 *  AS5600_SLOW_FILTER_2X
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::setSlowFilterVerify(AS5600SlowFilter _slow_filter) {
  AS5600::setSlowFilter(_slow_filter);
  return (AS5600::getSlowFilter() == _slow_filter) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить коэффициент медленной фильтрации 16х
 */
void AS5600::enableSlowFilter16x(void) {
  AS5600::setSlowFilter(AS5600_SLOW_FILTER_16X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 16х с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableSlowFilter16xVerify(void) {
  return AS5600::setSlowFilterVerify(AS5600_SLOW_FILTER_16X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 8х
 */
void AS5600::enableSlowFilter8x(void) {
  AS5600::setSlowFilter(AS5600_SLOW_FILTER_8X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 8х с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableSlowFilter8xVerify(void) {
  return AS5600::setSlowFilterVerify(AS5600_SLOW_FILTER_8X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 4х
 */
void AS5600::enableSlowFilter4x(void) {
  AS5600::setSlowFilter(AS5600_SLOW_FILTER_4X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 4х с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableSlowFilter4xVerify(void) {
  return AS5600::setSlowFilterVerify(AS5600_SLOW_FILTER_4X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 2х
 */
void AS5600::enableSlowFilter2x(void) {
  AS5600::setSlowFilter(AS5600_SLOW_FILTER_2X);
}
/*
 * @brief: включить коэффициент медленной фильтрации 2х с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableSlowFilter2xVerify(void) {
  return AS5600::setSlowFilterVerify(AS5600_SLOW_FILTER_2X);
}
/*
 * @brief: получить значение порога быстрой фильтрации
 * @return:
 *  AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5600_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_10_LSB
 */
AS5600FastFilterThreshold AS5600::getFastFilterThreshold(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  return (AS5600FastFilterThreshold)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_FTH_0) & 0x07); // 0x07=0b00000111
}
/*
 * @brief: установить новое значение порога быстрой фильтрации
 * @param _fast_filter_thredhold:
 *  AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5600_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_10_LSB
 */
void AS5600::setFastFilterThreshold(AS5600FastFilterThreshold _fast_filter_thredhold) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_H, conf_h_raw |= (_fast_filter_thredhold << AS5600_CONF_BIT_FTH_0));
}
/*
 * @brief: установить новое значение порога быстрой фильтраци с подтверждением
 * @param _fast_filter_thredhold:
 *  AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY
 *  AS5600_FAST_FILTER_THRESHOLD_6_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_7_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_9_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_18_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_21_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_24_LSB
 *  AS5600_FAST_FILTER_THRESHOLD_10_LSB
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::setFastFilterThresholdVerify(AS5600FastFilterThreshold _fast_filter_thredhold) {
  AS5600::setFastFilterThreshold(_fast_filter_thredhold);
  return (AS5600::getFastFilterThreshold() == _fast_filter_thredhold) ? AS5600_DEFAULT_REPORT_OK : AS5600_DEFAULT_REPORT_ERROR;
}
/*
 * @brief: включить только медленную фильтрацию
 */
void AS5600::enableSlowFilterOnly(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY);
}
/*
 * @brief: включить только медленную фильтрацию с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableSlowFilterOnlyVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY);
}
/*
 * @brief: включить быструю фильтрацию с порогом 6 LSB
 */
void AS5600::enableFastFilterThreshold6LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_6_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 6 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold6LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_6_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 7 LSB
 */
void AS5600::enableFastFilterThreshold7LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_7_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 7 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold7LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_7_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 9 LSB
 */
void AS5600::enableFastFilterThreshold9LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_9_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 9 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold9LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_9_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 18 LSB
 */
void AS5600::enableFastFilterThreshold18LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_18_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 18 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold18LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_18_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 21 LSB
 */
void AS5600::enableFastFilterThreshold21LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_21_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 21 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold21LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_21_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 24 LSB
 */
void AS5600::enableFastFilterThreshold24LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_24_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 24 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold24LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_24_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 10 LSB
 */
void AS5600::enableFastFilterThreshold10LSB(void) {
  AS5600::setFastFilterThreshold(AS5600_FAST_FILTER_THRESHOLD_10_LSB);
}
/*
 * @brief: включить быструю фильтрацию с порогом 10 LSB с подтверждением
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - не удалось установить
 *  AS5600_DEFAULT_REPORT_OK - удалось установить
 */
bool AS5600::enableFastFilterThreshold10LSBVerify(void) {
  return AS5600::setFastFilterThresholdVerify(AS5600_FAST_FILTER_THRESHOLD_10_LSB);
}
/*
 * @brief: проверить состояние бита сторожевого таймера. бит (WD:13) регистра CONF(13:0)
 * @return:
 *  AS5600_WATCHDOG_OFF - сторожевой таймер выключен
 *  AS5600_WATCHDOG_ON - сторожевой таймер включен
 */
bool AS5600::isWatchdog(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  return (bool)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_WD) & 0x01);
}
/*
 * @brief: включить сторожевой таймер. бит (WD:13) регистра CONF(13:0)
 */
void AS5600::enableWatchdog(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_H, conf_h_raw |= (1 << AS5600_CONF_BIT_WD));
}
/*
 * @brief: включить сторожевой таймер с подтверждением. бит (WD:13) регистра CONF(13:0)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - включение не удалось
 *  AS5600_DEFAULT_REPORT_OK - включение удалось
 */
bool AS5600::enableWatchdogVerify(void) {
  AS5600::enableWatchdog();
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  return (bool)((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_WD) & 0x01);
}
/*
 * @brief: выключить сторожевой таймер. бит (WD:13) регистра CONF(13:0)
 */
void AS5600::disableWatchdog(void) {
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  uint8_t conf_h_raw = AS5600::AS_RequestSingleRegister();
  AS5600::AS_WriteOneByte(AS5600_CONFIG_REG_CONF_H, conf_h_raw &= ~(1 << AS5600_CONF_BIT_WD));
}
/*
 * @brief: выключить сторожевой таймер с подтверждением. бит (WD:13) регистра CONF(13:0)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - выключение не удалось
 *  AS5600_DEFAULT_REPORT_OK - выключение удалось
 */
bool AS5600::disableWatchdogVerify(void) {
  AS5600::disableWatchdog();
  AS5600::AS_SendFirstRegister(AS5600_CONFIG_REG_CONF_H);
  return (bool)(!((AS5600::AS_RequestSingleRegister() >> AS5600_CONF_BIT_WD) & 0x01));
}
/**************************/
/**** OUTPUT REGISTERS ****/
/**************************/
/* 
 * @brief: получить чистое значение угла из регистра RAW ANGLE(11:0)
 * @return:
 *  0 - 4095
 */
word AS5600::getRawAngle(void) {
  AS5600::AS_SendFirstRegister(AS5600_OUT_REG_RAW_ANGLE_H);
  return AS5600::AS_RequestPairRegisters();
}
/* 
 * @brief: получить значение угла в градусах
 * @return:
 *  0.00 - 360.00
 */
float AS5600::getDegreesAngle(void) {
  return ((float)AS5600::getRawAngle() * 360) / 4096;
}
/* 
 * @brief: получить значение угла в радианах
 * @return:
 *  0.00 - 6.29
 */
float AS5600::getRadiansAngle(void) {
  return (AS5600::getDegreesAngle() * M_PI) / 180;
}
/* 
 * @brief: получить масштабированное значение угла из регистра ANGLE(11:0)
 * @note: учитываются значения в регистрах ZPOS, MPOS, MANG
 * @return:
 *  0 - 4095
 */
word AS5600::getScaledAngle(void) {
  AS5600::AS_SendFirstRegister(AS5600_OUT_REG_ANGLE_H);
  return AS5600::AS_RequestPairRegisters();
}
/**************************/
/**** STATUS REGISTERS ****/
/**************************/
/*
 * @brief: получить значение регистра STATUS
 * @return:
 *  AS5600_STATUS_REPORT_MD0_ML0_MH_0 - MD = 0, ML = 0, MH = 0
 *  AS5600_STATUS_REPORT_MD0_ML1_MH_0 - MD = 0, ML = 1, MH = 0
 *  AS5600_STATUS_REPORT_MD1_ML0_MH_0 - MD = 1, ML = 0, MH = 0
 *  AS5600_STATUS_REPORT_MD1_ML0_MH_1 - MD = 1, ML = 0, MH = 1
 *  AS5600_STATUS_REPORT_MD1_ML1_MH_0 - MD = 1, ML = 1, MH = 0
 */
AS5600StatusReports AS5600::getStatus(void) {
  AS5600::AS_SendFirstRegister(AS5600_STATUS_REG);
  return (AS5600StatusReports)AS5600::AS_RequestSingleRegister();
}
/*
 * @brief: определить наличие магнита. регистр STATUS (MD:5)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - магнита не обнаружен
 *  AS5600_DEFAULT_REPORT_OK - магнит обнаружен
 */
bool AS5600::isMagnetDetected(void) {
  return (bool)((AS5600::getStatus() >> AS5600_STATUS_BIT_MD_5) & 0x01);
}
/*
 * @brief: определить слишком слабый магнит. регистр STATUS (ML:4)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - магнит не слишком слабый
 *  AS5600_DEFAULT_REPORT_OK - магнит слишком слабый
 */
bool AS5600::isMagnetTooWeak(void) {
  return (bool)((AS5600::getStatus() >> AS5600_STATUS_BIT_ML_4) & 0x01);
}
/*
 * @brief: определить слишком сильный магнит. регистр STATUS (MH:3)
 * @return:
 *  AS5600_DEFAULT_REPORT_ERROR - магнит не слишком сильный
 *  AS5600_DEFAULT_REPORT_OK - магнит слишком сильный
 */
bool AS5600::isMagnetTooStrong(void) {
  return (bool)((AS5600::getStatus() >> AS5600_STATUS_BIT_MH_3) & 0x01);
}
/*
 * @brief: получить значение автоматического усиления из регистра AGC(7:0)
 * @return:
 *  0 - 255, при VCC = 5V
 *  0 - 128, при VCC = 3.3V
 */
byte AS5600::getAutomaticGainControl(void) {
  AS5600::AS_SendFirstRegister(AS5600_STATUS_REG_AGC);
  return AS5600::AS_RequestSingleRegister();
}
/* 
 * @brief: получить значение магнитуды из регистра MAGNITUDE(11:0)
 * @return:
 *  0 - 4095
 */
word AS5600::getMagnitude(void) {
  AS5600::AS_SendFirstRegister(AS5600_STATUS_REG_MAGNITUDE_H);
  return AS5600::AS_RequestPairRegisters();
}
/************************/
/**** BURN REGISTERS ****/
/************************/
/* 
 * @brief: записать НАВСЕГДА установленные значения в регистрах ZPOS(11:0) и MPOS(11:0)
 * @note: ВЫПОЛНИТЬ ЭТУ КОМАНДУ МОЖНО ТОЛЬКО 3(ТРИ) РАЗА ДЛЯ ОДНОГО ДАТЧИКА 
 *  ПРИ НАЛИЧИИ МАГНИТА (MD:5 = 1) И ПРИ НАЛИЧИИ РЕСУРСА В ZMCO(1:0)!
 * @param _use_special_verify:
 *  AS5600_FLAG_SPECIAL_VERIFY_DISABLE
 *  AS5600_FLAG_SPECIAL_VERIFY_ENABLE
 * @return:
 *  AS5600_BURN_REPROT_MAGNET_NOT_FOUND
 *  AS5600_BURN_REPROT_WRITE_OK
 *  AS5600_BURN_REPROT_WRITE_WRONG
 *  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY
 *  AS5600_BURN_REPROT_ZPOS_MPOS_NOT_SET
 *  AS5600_BURN_REPROT_ATTEMPTS_ENDED
 */
AS5600BurnReports AS5600::burnZeroAndMaxPositions(AS5600SpecialVerifyFlags _use_special_verify) {
  AS5600BurnReports result = AS5600_BURN_REPROT_MAGNET_NOT_FOUND;
  // Собираем значениях из критически выжных регистров
  byte burn_count = AS5600::getBurnPositionsCount();
  word z_pos = AS5600::getZeroPosition();
  word m_pos = AS5600::getMaxPosition();

  if(burn_count < AS5600_MAX_VALUE_ZMCO) { // Если ресурс для записи не исчерпан
    if(z_pos && m_pos) { // Если значения начального и максимального положения не 0
      // Наличие магнита проверяем НА ПОСЛЕДНЕМ ШАГЕ, перед отправлением команды на запись!
      if(AS5600::isMagnetDetected()) { // Если магнит обнаружен
        AS5600::AS_WriteOneByte(AS5600_BURN_REG, AS5600_CMD_BURN_ANGLE); // Отправляем команду записи
        if(_use_special_verify) { // Если используется проверка записанного
          AS5600::loadSavedValues(); // Загружаем из памяти ранее записанные данные
          // Получаем загруженные данные для сравнения
          word z_pos_now = AS5600::getZeroPosition();
          word m_pos_now = AS5600::getMaxPosition();
          if(z_pos == z_pos_now && m_pos == m_pos_now) { // Если записываемые данные совпадают с сохраненными
            result = AS5600_BURN_REPROT_WRITE_OK;
          }else {
            result = AS5600_BURN_REPROT_WRITE_WRONG;
          }
        }else {
          result = AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY;
        }
      }else {
        result = AS5600_BURN_REPROT_MAGNET_NOT_FOUND;
      }
    }else {
      result = AS5600_BURN_REPROT_ZPOS_MPOS_NOT_SET;
    }
  }else {
    result = AS5600_BURN_REPROT_ATTEMPTS_ENDED;
  }
  
  return result;
}
/* 
 * @brief: записать НАВСЕГДА установленные значения в регистрах MANG(11:0) и CONF(13:0)
 * @note: ВЫПОЛНИТЬ ЭТУ КОМАНДУ МОЖНО ТОЛЬКО 1(ОДИН) РАЗ ДЛЯ ОДНОГО ДАТЧИКА 
 *  ПРИ НАЛИЧИИ МАГНИТА (MD:5 = 1) И ПРИ НАЛИЧИИ РЕСУРСА В ZMCO(1:0)!
 * @param _use_special_verify:
 *  AS5600_FLAG_SPECIAL_VERIFY_DISABLE
 *  AS5600_FLAG_SPECIAL_VERIFY_ENABLE
 * @return:
 *  AS5600_BURN_REPROT_MAGNET_NOT_FOUND
 *  AS5600_BURN_REPROT_WRITE_OK
 *  AS5600_BURN_REPROT_WRITE_WRONG
 *  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY
 *  AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL
 *  AS5600_BURN_REPROT_ATTEMPTS_ENDED
 */
AS5600BurnReports AS5600::burnMaxAngleAndConfigurationValue(AS5600SpecialVerifyFlags _use_special_verify) {
  AS5600BurnReports result = AS5600_BURN_REPROT_MAGNET_NOT_FOUND;
  // Собираем значениях из критически выжных регистров
  byte burn_count = AS5600::getBurnPositionsCount();
  word m_ang = AS5600::getMaxAngle();
  word conf = AS5600::getRawConfigurationValue();
  
  if(burn_count == 0) { // Если ресурс для записи не исчерпан
    if(AS5600::getMaxAngle() >= AS5600_MIN_ANGLE_VALUE_DEC) { // Если значение угла подходит
      // Наличие магнита проверяем НА ПОСЛЕДНЕМ ШАГЕ, перед отправлением команды на запись!
      if(AS5600::isMagnetDetected()) { // Если магнит обнаружен
        AS5600::AS_WriteOneByte(AS5600_BURN_REG, AS5600_CMD_BURN_SETTINGS); // Отправляем команду записи настроек
        if(_use_special_verify) { // Если используется проверка записанного
          AS5600::loadSavedValues(); // Загружаем из памяти ранее записанные данные
          // Получаем загруженные данные для сравнения
          word m_ang_now = AS5600::getMaxAngle();
          word conf_now = AS5600::getRawConfigurationValue();
          if(m_ang == m_ang_now && conf == conf_now) { // Если записываемые данные совпадают с сохраненными
            result = AS5600_BURN_REPROT_WRITE_OK;
          }else {
            result = AS5600_BURN_REPROT_WRITE_WRONG;
          }
        }else {
          result = AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY;
        }
      }else {
        result = AS5600_BURN_REPROT_MAGNET_NOT_FOUND;
      }
    }else {
      result = AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL;
    }
  }else {
    result = AS5600_BURN_REPROT_ATTEMPTS_ENDED;
  }

  return result;
}
