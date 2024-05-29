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

#pragma once
#include "AMS_AS5600.h"

/*=== Зарезервированные I2C адреса для 7-ми битного формата ===*/
const uint8_t RESERVED_I2C_ADDR_L = 0x07;
const uint8_t RESERVED_I2C_ADDR_H = 0x78;

/*=== I2C адрес датчика по умолчанию ===*/
const uint8_t AS5600L_DEFAULT_I2C_ADDRESS = 0x40;

/*=== Некоторые адреса регистров датчика ===*/
/* Configuration Registers */
const uint8_t AS5600L_CONFIG_REG_I2CADDR = 0x20;
const uint8_t AS5600L_CONFIG_REG_I2CUPDT = 0x21; // он же I2CSTRB


class AS5600L : public AS5600 {
  private:
    uint8_t _i2c_address_ = 0; // Хранит I2C адрес датчика

  protected:
    void AS_SendFirstRegister(uint8_t _reg_addr) override; // Отправить адрес регистра
	
    uint8_t AS_RequestSingleRegister(void) override; // Запрос значения регистра размером 1 байт
    uint16_t AS_RequestPairRegisters(void) override; // Запрос значения регистра размером 2 байта
	
    void AS_WriteOneByte(uint8_t _reg, uint8_t _payload) override; // Запись одного байта в однобайтовый регистр
    void AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload) override; // Запись двух байтов в двубайтовый регистр
	
  public:
    AS5600L(TwoWire* _twi, uint8_t _iic_address = AS5600L_DEFAULT_I2C_ADDRESS); // Конструктор с использованием интерфейса и адреса I2C (адрес по умолчанию 0x40)
	
    void loadSavedValues(void) override; // Метод производителя для загрузки значений из памяти в регистры ZPOS, MPOS, MANG, CONF, I2CADDR
	
    bool isConnected(void) override; // Проверка по стандартному алгоритму поиска устройств на шине I2C

    virtual byte findDevice(void); // Получить первый доступный адрес на шине I2C. Адрес в конструкторе и методах работы с адресами не нужен
    virtual void findDevice(byte &_i2c_address); // Тоже самое, но через ссылку
	
    /* Configuration Registers */
    /** Управление регистром I2C адреса I2CADDR **/
    virtual byte getRegisterAddressI2C(void); // Получить значение I2C адреса из регистра I2CADDR
    virtual void getRegisterAddressI2C(byte &_i2c_address); // Тоже самое, но через ссылку
    virtual void setRegisterAddressI2C(byte _new_i2c_address); // Установить новое значение I2C адреса в регистр I2CADDR
    virtual bool setRegisterAddressI2CVerify(byte _new_i2c_address); // Тоже самое, но с подтверждением
    /** Управление регистром I2C адреса I2CUPDT (I2CSTRB) **/
    virtual byte getRegisterUpdateI2C(void); // Получить значение I2C адреса из регистра I2CUPDT (I2CSTRB)
    virtual void getRegisterUpdateI2C(byte &_i2c_address); // Тоже самое, но через ссылку
    virtual void setRegisterUpdateI2C(byte _new_i2c_address); // Установить новое значение I2C адреса в регистр I2CUPDT (I2CSTRB)
    virtual bool setRegisterUpdateI2CVerify(byte _new_i2c_address); // Тоже самое, но с подтверждением
	
    // Установка и получение I2C адреса датчика
    virtual byte getAddressI2C(void); // Получить нынешнее значение адреса датчика на шине I2C
    virtual void getAddressI2C(byte &_i2c_address); // Тоже самое, но через ссылку
    virtual void setAddressI2C(byte _new_i2c_address); // Установить новое значение адреса датчика на шине I2C
    virtual bool setAddressI2CVerify(byte _new_i2c_address); // Тоже самое, но с подтверждением

    /* Burn Command */
    AS5600BurnReports burnMaxAngleAndConfigurationValue(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE) override; // Записать навсегда MANG, CONF, I2CADDR. CMD_BURN_SETTINGS [1 РАЗ МАКСИМУМ!]
};
