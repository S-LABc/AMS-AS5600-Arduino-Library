/* 
 * Класс для Arduino IDE реализующий множество методов
 * взаимодействия с бесконтактным датчиком положения
 * AS5600 от компании AMS https://ams-osram.com/
 * 
 * Документация к датчику:
 ** https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
 ** https://ams-osram.com/products/sensors/position-sensors/ams-as5600-position-sensor
 *
 * Больше информации в WiKi:
 * https://github.com/S-LABc/AMS-AS5600-Arduino-Library/wiki
 * 
 * Контакты:
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2024. v2.0 / License MIT / Скляр Роман S-LAB
 */

#pragma once
#include "Arduino.h"
#include "Wire.h"

/*=== Настройки шины I2C датчика ===*/
const uint32_t AS5600_I2C_CLOCK_100KHZ = 100000UL;
const uint32_t AS5600_I2C_CLOCK_400KHZ = 400000UL;
const uint32_t AS5600_I2C_CLOCK_1MHZ   = 1000000UL;
const uint8_t AS5600_I2C_ADDRESS       = 0x36;

/*=== Выводы для DIR на разных платах (зависит от ядра) ===*/
#define STM32_AS5600_DEF_PIN   PC13
#define ESP8266_AS5600_DEF_PIN 2
#define ESP32_AS5600_DEF_PIN   4
#define ARDUINO_AS5600_DEF_PIN 3

// Количество регистров в памяти датчика для чтения
const uint8_t AS5600_REGISTER_MAP_SIZE = 0x12;

/*=== Адреса регистров датчика ===*/
/* Configuration Registers */
const uint8_t AS5600_CONFIG_REG_ZMCO   = 0x00;
const uint8_t AS5600_CONFIG_REG_ZPOS_H = 0x01;
const uint8_t AS5600_CONFIG_REG_ZPOS_L = 0x02;
const uint8_t AS5600_CONFIG_REG_MPOS_H = 0x03;
const uint8_t AS5600_CONFIG_REG_MPOS_L = 0x04;
const uint8_t AS5600_CONFIG_REG_MANG_H = 0x05;
const uint8_t AS5600_CONFIG_REG_MANG_L = 0x06;
const uint8_t AS5600_CONFIG_REG_CONF_H = 0x07;
const uint8_t AS5600_CONFIG_REG_CONF_L = 0x08;
// Значимые биты регистра CONF_H
enum AS5600ConfHighRegisterBits {
  AS5600_CONF_BIT_SF_0,
  AS5600_CONF_BIT_SF_1,
  AS5600_CONF_BIT_FTH_0,
  AS5600_CONF_BIT_FTH_1,
  AS5600_CONF_BIT_FTH_2,
  AS5600_CONF_BIT_WD,
};
// Значимые биты регистра CONF_L
enum AS5600ConfLowRegisterBits {
  AS5600_CONF_BIT_PM_0,
  AS5600_CONF_BIT_PM_1,
  AS5600_CONF_BIT_HYST_0,
  AS5600_CONF_BIT_HYST_1,
  AS5600_CONF_BIT_OUTS_0,
  AS5600_CONF_BIT_OUTS_1,
  AS5600_CONF_BIT_PWMF_0,
  AS5600_CONF_BIT_PWMF_1,
};
/* Output Registers */
const uint8_t AS5600_OUT_REG_RAW_ANGLE_H = 0x0C;
const uint8_t AS5600_OUT_REG_RAW_ANGLE_L = 0x0D;
const uint8_t AS5600_OUT_REG_ANGLE_H = 0x0E;
const uint8_t AS5600_OUT_REG_ANGLE_L = 0x0F;
/* Status Registers */
const uint8_t AS5600_STATUS_REG = 0x0B;
const uint8_t AS5600_STATUS_REG_AGC = 0x1A;
const uint8_t AS5600_STATUS_REG_MAGNITUDE_H = 0x1B;
const uint8_t AS5600_STATUS_REG_MAGNITUDE_L = 0x1C;
// Значимые биты регистра STATUS
enum AS5600StatusRegisterBits {
  AS5600_STATUS_BIT_MH_3 = 3,
  AS5600_STATUS_BIT_ML_4,
  AS5600_STATUS_BIT_MD_5,
};
/* Burn Commands */
const uint8_t AS5600_BURN_REG = 0xFF;
// Команды регистра BURN
const uint8_t AS5600_CMD_BURN_ANGLE = 0x80;
const uint8_t AS5600_CMD_BURN_SETTINGS = 0x40;
// Option A: Angle Programming Through the I²C Interface (Step 7)
// Option C: Programming a Maximum Angular Range Through the I²C Interface (Step 4)
const uint8_t AS5600_CMD_BURN_LOAD_OTP_CONTENT_0 = 0x01;
const uint8_t AS5600_CMD_BURN_LOAD_OTP_CONTENT_1 = 0x11;
const uint8_t AS5600_CMD_BURN_LOAD_OTP_CONTENT_2 = 0x10;

/*=== Вспомогательные значения ===*/
// Предельное значение регистра CONF_ZMCO
const uint8_t AS5600_MAX_VALUE_ZMCO = 0x03;
// Минимальный угол 18 градусов, примерно 205
const uint8_t AS5600_MIN_ANGLE_VALUE_DEC = 205; // 4096 /360 * 18 = 204.8
// Ответы стандартного вида успех/ошибка
const uint8_t AS5600_DEFAULT_REPORT_ERROR = 0;
const uint8_t AS5600_DEFAULT_REPORT_OK    = 1;
// Состояние сторожевого таймера
const uint8_t AS5600_WATCHDOG_OFF = 0;
const uint8_t AS5600_WATCHDOG_ON  = 1;
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
// Положительное направление вращения 
enum AS5600DirectionPolarity {
  AS5600_DIRECTION_POLARITY_CLOCKWISE, // По часовой стрелке (LOW - 0)
  AS5600_DIRECTION_POLARITY_COUNTERCLOCKWISE, // Против часовй стрелки (HIGH - 1)
};
// Расшифровка результата метода getStatus
enum AS5600StatusReports {
  AS5600_STATUS_REPORT_MD0_ML0_MH_0,
  AS5600_STATUS_REPORT_MD0_ML1_MH_0 = 2,
  AS5600_STATUS_REPORT_MD1_ML0_MH_0 = 4,
  AS5600_STATUS_REPORT_MD1_ML0_MH_1,
  AS5600_STATUS_REPORT_MD1_ML1_MH_0,
};
// Флаги для использования с методами из Burn Commands
enum AS5600SpecialVerifyFlags {
  AS5600_FLAG_SPECIAL_VERIFY_DISABLE,
  AS5600_FLAG_SPECIAL_VERIFY_ENABLE,
};
// Ответы методов burnZeroAndMaxPositions, burnMaxAngleAndConfigurationValue
enum AS5600BurnReports {
  AS5600_BURN_REPROT_SENSOR_NOT_CONNECTED,
  AS5600_BURN_REPROT_MAGNET_NOT_FOUND,
  AS5600_BURN_REPROT_WRITE_OK,
  AS5600_BURN_REPROT_WRITE_WRONG,
  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY,
  AS5600_BURN_REPROT_ZPOS_MPOS_NOT_SET,
  AS5600_BURN_REPROT_ATTEMPTS_ENDED,
  AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL,
  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_MAXANGLE,
  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY_WITHOUT_MAXANGLE,
};
// Хранит параметры виртуальной кнопки
struct AS5600Button {
  bool falg_button_state = false;
  byte minimum_agc = 0;
  byte maximum_agc = 0;
  byte deviation = 0;
};


class AS5600 {
  private:
    int8_t _pin_direction_ = -1; // Контакт микроконтроллера к которому подключен вывод DIR датчика
    AS5600Button _virtual_button_; // Структура с параметрами виртуальной кнопки

  protected:
    TwoWire* _wire_; // Объект для использования методов I2C

    virtual void AS_SendFirstRegister(uint8_t _reg_addr); // Отправить адрес регистра
	
    virtual uint8_t AS_RequestSingleRegister(void); // Запрос значения регистра размером 1 байт
    virtual uint16_t AS_RequestPairRegisters(void); // Запрос значения регистра размером 2 байта
	
    virtual void AS_WriteOneByte(uint8_t _reg, uint8_t _payload); // Запись одного байта в однобайтовый регистр
    virtual void AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload); // Запись двух байтов в двубайтовый регистр
	
  public:
    AS5600(TwoWire* _twi); // Конструктор с использованием только интерфейса I2C

    virtual void begin(void); // Вызов Wire.begin()
#if defined(ESP8266) || defined(ESP32) || defined(ARDUINO_ARCH_STM32)
    virtual void begin(int8_t _sda_pin, int8_t _scl_pin); // Вызов Wire.begin(SDA, SCL) с указанием выводов
#endif

    virtual void setClock(uint32_t _freq_hz = AS5600_I2C_CLOCK_400KHZ); // Настройка частоты на 100кГц, 400кГц, 1МГц, или пользовательское значение (по умолчанию 400кГц)
	
    virtual void loadSavedValues(void); // Метод производителя для загрузки значений из памяти в регистры ZPOS, MPOS, MANG, CONF
	
    virtual bool isConnected(void); // Проверка по стандартному алгоритму поиска устройств на шине I2C

    virtual bool getAllRegisters(byte *_registers, byte _array_size); // Получить значения всех регистров датчика в виде массива из 18 байт через ссылку
	
    /* Виртуальная кнопка */
    /** Настройки **/
    virtual void setButtonMinAGC(byte _btn_min_agc); // Установить новое минимальное значение срабатывания кнопки
    virtual byte getButtonMinAGC(void); // Получить минимальное значение срабатывания кнопки
    virtual void setButtonMaxAGC(byte _btn_max_agc); // Установить новое максимальное значение срабатывания кнопки
    virtual byte getButtonMaxAGC(void); // Получить максимальное значение срабатывания кнопки
    virtual void setButtonDeviation(byte _btn_div); // Установить новое значение отклонения срабатывания кнопки
    virtual byte getButtonDeviation(void); // Получить значение отклонения срабатывания кнопки
    /** События **/
    virtual bool isButtonPressed(void); // Проверка виртуальной кнопки на состояние НАЖАТА
    virtual bool isButtonReleased(void); // Проверка виртуальной кнопки на состояние ОТПУЩЕНА
	
    /* Управление контактом DIR датчика */
    virtual void attachDirectionPin(byte _pin_dir); // Назначить контакт микроконтроллера для управления положительным направлением вращения
    virtual void detachDirectionPin(void); // Освоободить назначенный контакт микроконтроллера для управления положительным направлением вращения
    virtual bool setDirection(AS5600DirectionPolarity _direction_polarity); // Установить положительное направление вращения (по/против часовой стрелки)
    virtual bool reverseDirection(void); // Изменить положительное направление вращения на противоположное
    virtual bool getDirection(void); // Получить текущее положительное направление вращения
	
    /* Configuration Registers */
    virtual byte getBurnPositionsCount(void); // Получить количество записей значений в ZPOS и MPOS (с завода ZMCO = 00). 0 - 3
    virtual void getBurnPositionsCount(byte &_burn_positions_count); // Тоже самое, но через ссылку
	
    virtual word getZeroPosition(void); // Получить значение начального положения (начальный угол). 0 - 4095
    virtual void getZeroPosition(word &_zero_position); // Тоже самое, но через ссылку
    virtual void setZeroPosition(word _zero_position); // Установить новое начальное положение ZPOS
    virtual bool setZeroPositionVerify(word _zero_position); // Тоже самое, но с подтверждением
    virtual void setZeroPositionViaRawAngle(void); // Установить новое начальное положение ZPOS используя нынешнее положение магнита (getRawAngle)
    virtual bool setZeroPositionViaRawAngleVerify(void); // Тоже самое, но с подтверждением
	
    virtual word getMaxPosition(void); // Получить значение конечного положения (конечный угол). 0 - 4095
    virtual void getMaxPosition(word &_max_position); // Тоже самое, но через ссылку
    virtual void setMaxPosition(word _max_position); // Установить новое конечное положение MPOS
    virtual bool setMaxPositionVerify(word _max_position); // Тоже самое, но с подтверждением
    virtual void setMaxPositionViaRawAngle(void); // Установить новое начальное положение MPOS используя нынешнее положение магнита (getRawAngle)
    virtual bool setMaxPositionViaRawAngleVerify(void); // Тоже самое, но с подтверждением
	
    virtual word getMaxAngle(void); // Получить значение максимально угла. 0 - 4095
    virtual void getMaxAngle(word &_max_angle); // Тоже самое, но через ссылку
    virtual void setMaxAngle(word _max_angle); // Установить новое значение максимального угла MANG
    virtual bool setMaxAngleVerify(word _max_angle); // Тоже самое, но с подтверждением
    virtual void setMaxAngleViaRawAngle(void); // Установить новое начальное положение MANG используя нынешнее положение магнита (getRawAngle)
    virtual bool setMaxAngleViaRawAngleVerify(void); // Тоже самое, но с подтверждением
	
    virtual word getRawConfigurationValue(void); // Получить "сырые" значения регистра конфигураций CONF. 0 - 4095
    virtual void getRawConfigurationValue(word &_confuration_value); // Тоже самое, но через ссылку
    virtual void setRawConfigurationValue(word _confuration_value); // Установить новые "сырые" значения регистра конфигураций CONF
    virtual bool setRawConfigurationValueVerify(word _confuration_value); // Тоже самое, но с подтверждением
    /** Управление Power Mode битами PM **/
    virtual AS5600PowerModes getPowerMode(void); // Получить текущий режим питания
    virtual void getPowerMode(AS5600PowerModes &_power_mode); // Тоже самое, но через ссылку
    virtual void setPowerMode(AS5600PowerModes _power_mode); // Установить новый режим питания
    virtual bool setPowerModeVerify(AS5600PowerModes _power_mode); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableNomPowerMode(void); // Включить нормальный режим питания
    virtual bool enableNomPowerModeVerify(void); // Тоже самое, но с подтверждением
    virtual void enableLowPowerMode1(void); // Включить пониженный режим питания 1
    virtual bool enableLowPowerMode1Verify(void); // Тоже самое, но с подтверждением
    virtual void enableLowPowerMode2(void); // Включить пониженный режим питания 2
    virtual bool enableLowPowerMode2Verify(void); // Тоже самое, но с подтверждением
    virtual void enableLowPowerMode3(void); // Включить пониженный режим питания 3
    virtual bool enableLowPowerMode3Verify(void); // Тоже самое, но с подтверждением
    /** Управление Hysteresis битами HYST **/
    virtual AS5600Hysteresis getHysteresis(void); // Получить параметры гистерезиса
    virtual void getHysteresis(AS5600Hysteresis &_hysteresis); // Тоже самое, но через ссылку
    virtual void setHysteresis(AS5600Hysteresis _hysteresis); // Установить новые параметры гистерезиса
    virtual bool setHysteresisVerify(AS5600Hysteresis _hysteresis); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void disableHysteresis(void); // Отключить гистерезис
    virtual bool disableHysteresisVerify(void); // Тоже самое, но с подтверждением
    virtual void enableHysteresis1LSB(void); // Включить гистерезис 1 LSB
    virtual bool enableHysteresis1LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableHysteresis2LSB(void); // Включить гистерезис 2 LSB
    virtual bool enableHysteresis2LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableHysteresis3LSB(void); // Включить гистерезис 3 LSB
    virtual bool enableHysteresis3LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Output Stage битами OUTS **/
    virtual AS5600OutputStage getOutputStage(void); // Получить режим работы контакта OUT
    virtual void getOutputStage(AS5600OutputStage &_output_stage); // Тоже самое, но через ссылку
    virtual void setOutputStage(AS5600OutputStage _output_stage); // Установить режим работы контакта OUT
    virtual bool setOutputStageVerify(AS5600OutputStage _output_stage); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableOutputAnalogFullRange(void); // OUT как аналоговый выход (0 - 100%)
    virtual bool enableOutputAnalogFullRangeVerify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputAnalogReducedRange(void); // OUT как аналоговый выход (10 - 90%)
    virtual bool enableOutputAnalogReducedRangeVerify(void); // Тоже самое, но с подтверждением
    virtual void enableOutputDigitalPWM(void); // OUT как цифровой ШИМ выход
    virtual bool enableOutputDigitalPWMVerify(void); // Тоже самое, но с подтверждением
    /** Управление PWM Frequency битами PWMF **/
    virtual AS5600PWMFrequency getPWMFrequency(void); // Получить значение частоты ШИМ
    virtual void getPWMFrequency(AS5600PWMFrequency &_pwm_frequency); // Тоже самое, но через ссылку
    virtual void setPWMFrequency(AS5600PWMFrequency _pwm_frequency); // Установить новое значение частоты ШИМ
    virtual bool setPWMFrequencyVerify(AS5600PWMFrequency _pwm_frequency); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enablePWMFrequency115Hz(void); // Включить ШИМ 115Гц
    virtual bool enablePWMFrequency115HzVerify(void); // Тоже самое, но с подтверждением
    virtual void enablePWMFrequency230Hz(void); // Включить ШИМ 230Гц
    virtual bool enablePWMFrequency230HzVerify(void); // Тоже самое, но с подтверждением
    virtual void enablePWMFrequency460Hz(void); // Включить ШИМ 460Гц
    virtual bool enablePWMFrequency460HzVerify(void); // Тоже самое, но с подтверждением
    virtual void enablePWMFrequency920Hz(void); // Включить ШИМ 920Гц
    virtual bool enablePWMFrequency920HzVerify(void); // Тоже самое, но с подтверждением
    /** Управление Slow Filter битами SF **/
    virtual AS5600SlowFilter getSlowFilter(void); // Получить коэффициент медленной фильтрации
    virtual void getSlowFilter(AS5600SlowFilter &_slow_filter); // Тоже самое, но через ссылку
    virtual void setSlowFilter(AS5600SlowFilter _slow_filter); // Установить новый коэффициент медленной фильтрации
    virtual bool setSlowFilterVerify(AS5600SlowFilter _slow_filter); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableSlowFilter16x(void); // Включить коэффициент 16х
    virtual bool enableSlowFilter16xVerify(void); // Тоже самое, но с подтверждением
    virtual void enableSlowFilter8x(void); // Включить коэффициент 8х
    virtual bool enableSlowFilter8xVerify(void); // Тоже самое, но с подтверждением
    virtual void enableSlowFilter4x(void); // Включить коэффициент 4х
    virtual bool enableSlowFilter4xVerify(void); // Тоже самое, но с подтверждением
    virtual void enableSlowFilter2x(void); // Включить коэффициент 2х
    virtual bool enableSlowFilter2xVerify(void); // Тоже самое, но с подтверждением
    /** Управление Fast Filter Threshold битами FTH **/
    virtual AS5600FastFilterThreshold getFastFilterThreshold(void); // Получить порог быстрой фильтрации
    virtual void getFastFilterThreshold(AS5600FastFilterThreshold &_fast_filter_thredhold); // Тоже самое, но через ссылку
    virtual void setFastFilterThreshold(AS5600FastFilterThreshold _fast_filter_thredhold); // Установить порог быстрой фильтрации
    virtual bool setFastFilterThresholdVerify(AS5600FastFilterThreshold _fast_filter_thredhold); // Тоже самое, но с подтверждением
    // Отдельные режимы
    virtual void enableSlowFilterOnly(void); // Включить только медленную фильтрацию
    virtual bool enableSlowFilterOnlyVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold6LSB(void); // Включить быструю фильтрацию 6 LSB
    virtual bool enableFastFilterThreshold6LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold7LSB(void); // Включить быструю фильтрацию 7 LSB
    virtual bool enableFastFilterThreshold7LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold9LSB(void); // Включить быструю фильтрацию 9 LSB
    virtual bool enableFastFilterThreshold9LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold18LSB(void); // Включить быструю фильтрацию 18 LSB
    virtual bool enableFastFilterThreshold18LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold21LSB(void); // Включить быструю фильтрацию 21 LSB
    virtual bool enableFastFilterThreshold21LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold24LSB(void); // Включить быструю фильтрацию 24 LSB
    virtual bool enableFastFilterThreshold24LSBVerify(void); // Тоже самое, но с подтверждением
    virtual void enableFastFilterThreshold10LSB(void); // Включить быструю фильтрацию 10 LSB
    virtual bool enableFastFilterThreshold10LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Watchdog битом WD **/
    virtual bool isWatchdog(void); // Определить состояние сторожевого таймера
    // Отдельные способы
    virtual void enableWatchdog(void); // Включить сторожевой таймер
    virtual bool enableWatchdogVerify(void); // Тоже самое, но с подтверждением
    virtual void disableWatchdog(void); // Выключить сторожевой таймер
    virtual bool disableWatchdogVerify(void); // Тоже самое, но с подтверждением
    
    /* Output Registers */
    virtual word getRawAngle(void); // Получить угол в чистом виде. 0 - 4095
    virtual void getRawAngle(word &_raw_angle); // Тоже самое, но через ссылку
    virtual float getDegreesAngle(void); // Получить угол в градусах. 0.00 - 360.00. Основан на значениях от getRawAngle
    virtual void getDegreesAngle(float &_degrees_angle); // Тоже самое, но через ссылку
    virtual float getRadiansAngle(void); // Получить угол в радианах 0.00 - 6.28. Основан на значениях от getRawAngle
    virtual void getRadiansAngle(float &_radians_angle); // Тоже самое, но через ссылку
    
    virtual word getScaledAngle(void); // Получить масштабированный угол с учетом ZPOS, MPOS или MANG. 0 - 4095
    virtual void getScaledAngle(word &_scaled_angle); // Тоже самое, но через ссылку
	
    /* Status Registers */
    virtual AS5600StatusReports getStatus(void); // Получить значение регистра STATUS
    virtual void getStatus(AS5600StatusReports &_status); // Тоже самое, но через ссылку
    virtual bool isMagnetDetected(void); // Определить наличие магнита MD
    virtual bool isMagnetTooWeak(void); // Определить очень слабый магнит ML
    virtual bool isMagnetTooStrong(void); // Определить очень сильный магнит MH
	
    virtual byte getAutomaticGainControl(void); // Получить значение автоусиления. При VCC = 5В -> 0 - 255, при VCC = 3.3В -> 0 - 128
    virtual void getAutomaticGainControl(byte &_agc); // Тоже самое, но через ссылку
	
    virtual word getMagnitude(void); // Получить значение магнитуды. 0 - 4095
    virtual void getMagnitude(word &_magnitude); // Тоже самое, но через ссылку
    
    /* Burn Commands */
    virtual AS5600BurnReports burnZeroAndMaxPositions(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда ZPOS, MPOS. CMD_BURN_ANGLE [3 РАЗА МАКСИМУМ!]
    virtual AS5600BurnReports burnMaxAngleAndConfigurationValue(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда MANG, CONF. CMD_BURN_SETTINGS [1 РАЗ МАКСИМУМ!]
};
