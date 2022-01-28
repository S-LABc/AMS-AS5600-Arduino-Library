/* 
 * Класс для Arduino IDE реализующий множество методов
 * взаимодействия с бесконтактным датчиком положения
 * AS5600 от компании AMS https://ams.com/ams-start
 * 
 * Документаци к датчику:
 ** https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
 ** https://ams.com/en/as5600
 * 
 * Контакты:
 ** YouTube - https://www.youtube.com/channel/UCbkE52YKRphgkvQtdwzQbZQ
 ** Telegram - https://www.t.me/slabyt
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2022. v1.0 / License MIT / Скляр Роман S-LAB
 */

#pragma once
#include "Arduino.h"
#include "Wire.h"

/*=== Настройки шины I2C датчика ===*/
#define AS5600_I2C_CLOCK 400000UL
#define AS5600_I2C_ADDRESS 0x36

/*=== Выводы на разных платах (зависит от ядра) ===*/
#define STM32_AS5600_DEF_PIN PC13
#define ESP32_AS5600_DEF_PIN 4
#define ARDUINO_AS5600_DEF_PIN 3

/*=== Адреса регистров датчика ===*/
/* Configuration Registers */
#define AS5600_CONFIG_REG_ZMCO 0x00
#define AS5600_CONFIG_REG_ZPOS_H 0x01
#define AS5600_CONFIG_REG_ZPOS_L 0x02
#define AS5600_CONFIG_REG_MPOS_H 0x03
#define AS5600_CONFIG_REG_MPOS_L 0x04
#define AS5600_CONFIG_REG_MANG_H 0x05
#define AS5600_CONFIG_REG_MANG_L 0x06
#define AS5600_CONFIG_REG_CONF_H 0x07
#define AS5600_CONFIG_REG_CONF_L 0x08
// Значимые биты регистра CONF
enum AS5600ConfRegisterBits {
  AS5600_CONF_BIT_PM_0,
  AS5600_CONF_BIT_PM_1,
  AS5600_CONF_BIT_HYST_0,
  AS5600_CONF_BIT_HYST_1,
  AS5600_CONF_BIT_OUTS_0,
  AS5600_CONF_BIT_OUTS_1,
  AS5600_CONF_BIT_PWMF_0,
  AS5600_CONF_BIT_PWMF_1,
  AS5600_CONF_BIT_SF_0,
  AS5600_CONF_BIT_SF_1,
  AS5600_CONF_BIT_FTH_0,
  AS5600_CONF_BIT_FTH_1,
  AS5600_CONF_BIT_FTH_2,
  AS5600_CONF_BIT_WD,
};
/* Output Registers */
#define AS5600_OUT_REG_RAW_ANGLE_H 0x0C
#define AS5600_OUT_REG_RAW_ANGLE_L 0x0D
#define AS5600_OUT_REG_ANGLE_H 0x0E
#define AS5600_OUT_REG_ANGLE_L 0x0F
/* Status Registers */
#define AS5600_STATUS_REG 0x0B
#define AS5600_STATUS_REG_AGC 0x1A
#define AS5600_STATUS_REG_MAGNITUDE_H 0x1B
#define AS5600_STATUS_REG_MAGNITUDE_L 0x1C
// Значимые биты регистра STATUS
enum AS5600StatusRegisterBits {
  AS5600_STATUS_BIT_MH_3 = 3,
  AS5600_STATUS_BIT_ML_4,
  AS5600_STATUS_BIT_MD_5,
};
/* Burn Commands */
#define AS5600_BURN_REG 0xFF
// Команды регистра BURN
#define AS5600_CMD_BURN_ANGLE 0x80
#define AS5600_CMD_BURN_SETTINGS 0x40
// Option A: Angle Programming Through the I²C Interface (Step 7)
#define AS5600_CMD_BURN_LOAD_OTP_CONTENT_0 0x01
#define AS5600_CMD_BURN_LOAD_OTP_CONTENT_1 0x11
#define AS5600_CMD_BURN_LOAD_OTP_CONTENT_2 0x10

/*=== Вспомогательные значения ===*/
// Предельное значение регистра CONF_ZMCO
#define AS5600_MAX_VALUE_ZMCO 0x03
// Минимальный угол 18 градусов, примерно 205
#define AS5600_MIN_ANGLE_VALUE_DEC 205
// Ответы стандартного вида успех/ошибка
#define AS5600_DEFAULT_REPORT_ERROR 0
#define AS5600_DEFAULT_REPORT_OK 1
// Режимы питания
enum AS5600PowerModes {
  AS5600_NOM_POWER_MODE,
  AS5600_LOW_POWER_MODE_1,
  AS5600_LOW_POWER_MODE_2,
  AS5600_LOW_POWER_MODE_3,
};
// Режимы гистерезиса
enum AS5600Hysteresis {
  AS5600_HYSTERESIS_OFF,
  AS5600_HYSTERESIS_1_LSB,
  AS5600_HYSTERESIS_2_LSB,
  AS5600_HYSTERESIS_3_LSB,
};
// Режимы вывода OUT
enum AS5600OutputStage {
  AS5600_OUTPUT_ANALOG_FULL_RANGE,
  AS5600_OUTPUT_ANALOG_REDUCED_RANGE,
  AS5600_OUTPUT_DIGITAL_PWM,
};
// Варианты частоты ШИМ
enum AS5600PWMFrequency {
  AS5600_PWM_FREQUENCY_115HZ,
  AS5600_PWM_FREQUENCY_230HZ,
  AS5600_PWM_FREQUENCY_460HZ,
  AS5600_PWM_FREQUENCY_920HZ,
};
// Шаги медленной фильтрации
enum AS5600SlowFilter {
  AS5600_SLOW_FILTER_16X,
  AS5600_SLOW_FILTER_8X,
  AS5600_SLOW_FILTER_4X,
  AS5600_SLOW_FILTER_2X,
};
// Пороги быстрой фильтрации
enum AS5600FastFilterThreshold {
  AS5600_FAST_FILTER_THRESHOLD_SLOW_FILTER_ONLY,
  AS5600_FAST_FILTER_THRESHOLD_6_LSB,
  AS5600_FAST_FILTER_THRESHOLD_7_LSB,
  AS5600_FAST_FILTER_THRESHOLD_9_LSB,
  AS5600_FAST_FILTER_THRESHOLD_18_LSB,
  AS5600_FAST_FILTER_THRESHOLD_21_LSB,
  AS5600_FAST_FILTER_THRESHOLD_24_LSB,
  AS5600_FAST_FILTER_THRESHOLD_10_LSB,
};
// Состояние сторожевого таймера
enum AS5600Watchdog {
  AS5600_WATCHDOG_OFF,
  AS5600_WATCHDOG_ON,
};
// Положительное направление вращения 
enum AS5600DirectionPolarity {
  AS5600_DIRECTION_POLARITY_CLOCKWISE, // По часовой стрелке (LOW - 0)
  AS5600_DIRECTION_POLARITY_COUNTERCLOCKWISE, // Против часовй стрелки (HIGH - 1)
};
// Флаги для использования с методами из Burn Commands
enum AS5600SpecialVerifyFlags {
  AS5600_FLAG_SPECIAL_VERIFY_DISABLE,
  AS5600_FLAG_SPECIAL_VERIFY_ENABLE,
};
// Расшифровка результатов методов burn
enum AS5600BurnReports {
  AS5600_BURN_REPROT_MAGNET_NOT_FOUND,
  AS5600_BURN_REPROT_WRITE_OK,
  AS5600_BURN_REPROT_WRITE_WRONG,
  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY,
  AS5600_BURN_REPROT_ZPOS_MPOS_NOT_SET,
  AS5600_BURN_REPROT_ATTEMPTS_ENDED,
  AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL,
};


class AS5600 {
  private:
    TwoWire *__wire; // Объект для использования методов I2C
    uint8_t _pin_direction_ = 0; // Контакт микроконтроллера к которому подключен вывод DIR датчика

  protected:
    void AS_SendFirstRegister(uint8_t _reg_addr); // Отправить адрес регистра
    uint8_t AS_RequestSingleRegister(void); // Запрос значения регистра размером 1 байт
    uint16_t AS_RequestPairRegisters(void); // Запрос значения регистра размером 2 байта
	
    void AS_WriteOneByte(uint8_t _reg, uint8_t _payload); // Запись одного байта в однобайтовый регистр
    void AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload); // Запись двух байтов в двубайтовый регистр
	
  public:
    AS5600(TwoWire *twi); // Конструктор с использованием только интерфейса I2C
    AS5600(TwoWire *twi, uint8_t _pin_dir, AS5600DirectionPolarity _def_polar_dir = AS5600_DIRECTION_POLARITY_CLOCKWISE); // Конструктор с использованием интерфейса I2C, контакта DIR(8), направлением вращения
    ~AS5600(); // Деструктор. Обнуляет _pin_direction

    void begin(void); // Вызов Wire.begin()
    void setClock(void); // Настройка частоты на 400кГц
    void loadSavedValues(void); // Метод производителя для загрузки значений из памяти в регистры ZPOS, MPOS, MANG, CONF
	
    uint8_t isConnected(void); // Проверка по стандартному алгоритму поиска устройств на линии I2C
    uint8_t setDirection(AS5600DirectionPolarity _polar_dir); // Установить положительное направление вращения (по/против часовой стрелки)
    uint8_t reverseDirection(void); // Изменить положительное направление вращения на противоположное
    uint8_t getDirection(void); // Получить текущее положительное направление вращения
	
    /* Configuration Registers */
    uint8_t getBurnPositionsCount(void); // Получить количество записей значений в ZPOS и MPOS (с завода ZMCO = 00). 0-3
	
    uint16_t getZeroPosition(void); // Получить значение начального положения ZPOS (начальный угол). 0-4095
    void setZeroPosition(uint16_t _start_angle); // Установить новое начальное положение ZPOS
    uint8_t setZeroPositionVerify(uint16_t _start_angle); // Тоже самое, но с подтверждением
	
    uint16_t getMaxPosition(void); // Получить значение конечного положения MPOS (конечный угол). 0-4095
    void setMaxPosition(uint16_t _end_angle); // Установить новое конечное положение MPOS
    uint8_t setMaxPositionVerify(uint16_t _end_angle); // Тоже самое, но с подтверждением
	
    uint16_t getMaxAngle(void); // Получить значение максимально угла MANG. 0-4095
    void setMaxAngle(uint16_t _max_angle); // Установить новое значение максимального угла MANG
    uint8_t setMaxAngleVerify(uint16_t _max_angle); // Тоже самое, но с подтверждением
	
    uint16_t getRawConfigurationValue(void); // Получить "сырые" значения регистра конфигураций CONF. 0-4095
    void setRawConfigurationValue(uint16_t _conf_value); // Установить новые "сырые" значения регистра конфигураций CONF
    uint8_t setRawConfigurationValueVerify(uint16_t _conf_value); // Тоже самое, но с подтверждением
    /** Управление Power Mode битами PM **/
    uint8_t getPowerMode(void); // Получить текущий режим питания
    void setPowerMode(AS5600PowerModes _pwr_mode); // Установить новый режим питания
    uint8_t setPowerModeVerify(AS5600PowerModes _pwr_mode); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableNomPowerMode(void); // Включить нормальный режим питания
    uint8_t enableNomPowerModeVerify(void); // Тоже самое, но с подтверждением
    void enableLowPowerMode1(void); // Включить пониженный режим питания 1
    uint8_t enableLowPowerMode1Verify(void); // Тоже самое, но с подтверждением
    void enableLowPowerMode2(void); // Включить пониженный режим питания 2
    uint8_t enableLowPowerMode2Verify(void); // Тоже самое, но с подтверждением
    void enableLowPowerMode3(void); // Включить пониженный режим питания 3
    uint8_t enableLowPowerMode3Verify(void); // Тоже самое, но с подтверждением
    /** Управление Hysteresis битами HYST **/
    uint8_t getHysteresis(void); // Получить параметры гистерезиса
    void setHysteresis(AS5600Hysteresis _hyst); // Установить новые параметры гистерезиса
    uint8_t setHysteresisVerify(AS5600Hysteresis _hyst); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void disableHysteresis(void); // Отключить гистерезис
    uint8_t disableHysteresisVerify(void); // Тоже самое, но с подтверждением
    void enableHysteresis1LSB(void); // Включить гистерезис 1 LSB
    uint8_t enableHysteresis1LSBVerify(void); // Тоже самое, но с подтверждением
    void enableHysteresis2LSB(void); // Включить гистерезис 3 LSB
    uint8_t enableHysteresis2LSBVerify(void); // Тоже самое, но с подтверждением
    void enableHysteresis3LSB(void); // Включить гистерезис 3 LSB
    uint8_t enableHysteresis3LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Output Stage битами OUTS **/
    uint8_t getOutputStage(void); // Получить режим работы контакта OUT
    void setOutputStage(AS5600OutputStage _out_stage); // Установить режим работы контакта OUT
    uint8_t setOutputStageVerify(AS5600OutputStage _out_stage); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableOutputAnalogFullRange(void); // OUT как аналоговый выход (0%-100%)
    uint8_t enableOutputAnalogFullRangeVerify(void); // Тоже самое, но с подтверждением
    void enableOutputAnalogReducedRange(void); // OUT как аналоговый выход (10%-90%)
    uint8_t enableOutputAnalogReducedRangeVerify(void); // Тоже самое, но с подтверждением
    void enableOutputDigitalPWM(void); // OUT как цифровой ШИМ выход
    uint8_t enableOutputDigitalPWMVerify(void); // Тоже самое, но с подтверждением
    /** Управление PWM Frequency битами PWMF **/
    uint8_t getPWMFrequency(void); // Получить значение частоты ШИМ
    void setPWMFrequency(AS5600PWMFrequency _frequency); // Установить новое значение частоты ШИМ
    uint8_t setPWMFrequencyVerify(AS5600PWMFrequency _frequency); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enablePWMFrequency115Hz(void); // Включить ШИМ 115Гц
    uint8_t enablePWMFrequency115HzVerify(void); // Тоже самое, но с подтверждением
    void enablePWMFrequency230Hz(void); // Включить ШИМ 230Гц
    uint8_t enablePWMFrequency230HzVerify(void); // Тоже самое, но с подтверждением
    void enablePWMFrequency460Hz(void); // Включить ШИМ 460Гц
    uint8_t enablePWMFrequency460HzVerify(void); // Тоже самое, но с подтверждением
    void enablePWMFrequency920Hz(void); // Включить ШИМ 920Гц
    uint8_t enablePWMFrequency920HzVerify(void); // Тоже самое, но с подтверждением
    /** Управление Slow Filter битами SF **/
    uint8_t getSlowFilter(void); // Получить коэффициент медленной фильтрации
    void setSlowFilter(AS5600SlowFilter _slow_filter); // Установить новый коэффициент медленной фильтрации
    uint8_t setSlowFilterVerify(AS5600SlowFilter _slow_filter); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableSlowFilter16x(void); // Включить коэффициент 16х
    uint8_t enableSlowFilter16xVerify(void); // Тоже самое, но с подтверждением
    void enableSlowFilter8x(void); // Включить коэффициент 8х
    uint8_t enableSlowFilter8xVerify(void); // Тоже самое, но с подтверждением
    void enableSlowFilter4x(void); // Включить коэффициент 4х
    uint8_t enableSlowFilter4xVerify(void); // Тоже самое, но с подтверждением
    void enableSlowFilter2x(void); // Включить коэффициент 2х
    uint8_t enableSlowFilter2xVerify(void); // Тоже самое, но с подтверждением
    /** Управление Fast Filter Threshold битами FTH **/
    uint8_t getFastFilterThreshold(void); // Получить порог быстрой фильтрации
    void setFastFilterThreshold(AS5600FastFilterThreshold _fast_filter_thredhold); // Установить порог быстрой фильтрации
    uint8_t setFastFilterThresholdVerify(AS5600FastFilterThreshold _fast_filter_thredhold); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableSlowFilterOnly(void); // Включить только медленную фильтрацию
    uint8_t enableSlowFilterOnlyVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold6LSB(void); // Включить быструю фильтрацию 6 LSB
    uint8_t enableFastFilterThreshold6LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold7LSB(void); // Включить быструю фильтрацию 7 LSB
    uint8_t enableFastFilterThreshold7LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold9LSB(void); // Включить быструю фильтрацию 9 LSB
    uint8_t enableFastFilterThreshold9LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold18LSB(void); // Включить быструю фильтрацию 18 LSB
    uint8_t enableFastFilterThreshold18LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold21LSB(void); // Включить быструю фильтрацию 21 LSB
    uint8_t enableFastFilterThreshold21LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold24LSB(void); // Включить быструю фильтрацию 24 LSB
    uint8_t enableFastFilterThreshold24LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold10LSB(void); // Включить быструю фильтрацию 10 LSB
    uint8_t enableFastFilterThreshold10LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Watchdog битом WD **/
    uint8_t isWatchdog(void); // Определить состояние сторожевого таймера
    // Отдельные способы
    void enableWatchdog(void); // Включить сторожевой таймер
    uint8_t enableWatchdogVerify(void); // Тоже самое, но с подтверждением
    void disableWatchdog(void); // Выключить сторожевой таймер
    uint8_t disableWatchdogVerify(void); // Тоже самое, но с подтверждением
    
    /* Output Registers */
    uint16_t getRawAngle(void); // Получить угол в чистом виде. 0-4095
    
    uint16_t getScaledAngle(void); // Получить масштабированный угол с учетом ZPOS, MPOS или MANG. 0-4095
	
    /* Status Registers */
    uint8_t getStatus(void); // Получить значение регистра STATUS (MD=1-магнит обнаружен, ML=1-магнит слабый, MH=1-магнит сильный)
    uint8_t isMagnetDetected(void); // Определить наличие магнита STATUS (MD=1-магнит обнаружен)
    uint8_t isMagnetTooWeak(void); // Определить очень слабый магнит STATUS (ML=1-магнит слабый)
    uint8_t isMagnetTooStrong(void); // Определить очень сильный магнит STATUS (MH=1-магнит сильный)
	
    uint8_t getAutomaticGainControl(void); // Получить значение автоусиления AGC (При 5В 0-255, при 3.3В 0-128)
	
    uint16_t getMagnitude(void); // Получить значение магнитуды. 0-4095
    
    /* Burn Commands */
    uint8_t burnZeroAndMaxPositions(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда ZPOS, MPOS; CMD_BURN_ANGLE [3 РАЗА МАКСИМУМ!]
    uint8_t burnMaxAngleAndConfigurationValue(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда MANG, CONF; CMD_BURN_SETTINGS [1 РАЗ МАКСИМУМ!]
};
