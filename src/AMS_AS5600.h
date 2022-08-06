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
 * Copyright (C) 2022. v1.4 / License MIT / Скляр Роман S-LAB
 */

#pragma once
#include "Arduino.h"
#include "Wire.h"

/*=== Настройки шины I2C датчика ===*/
#define AS5600_I2C_CLOCK_100KHZ 100000UL
#define AS5600_I2C_CLOCK_400KHZ 400000UL
#define AS5600_I2C_CLOCK_1MHZ   1000000UL
#define AS5600_I2C_ADDRESS      0x36

/*=== Выводы на разных платах (зависит от ядра) ===*/
#define STM32_AS5600_DEF_PIN   PC13
#define ESP32_AS5600_DEF_PIN   4
#define ARDUINO_AS5600_DEF_PIN 3

/*=== Адреса регистров датчика ===*/
/* Configuration Registers */
#define AS5600_CONFIG_REG_ZMCO   0x00
#define AS5600_CONFIG_REG_ZPOS_H 0x01
#define AS5600_CONFIG_REG_ZPOS_L 0x02
#define AS5600_CONFIG_REG_MPOS_H 0x03
#define AS5600_CONFIG_REG_MPOS_L 0x04
#define AS5600_CONFIG_REG_MANG_H 0x05
#define AS5600_CONFIG_REG_MANG_L 0x06
#define AS5600_CONFIG_REG_CONF_H 0x07
#define AS5600_CONFIG_REG_CONF_L 0x08
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
// Option C: Programming a Maximum Angular Range Through the I²C Interface (Step 4)
#define AS5600_CMD_BURN_LOAD_OTP_CONTENT_0 0x01
#define AS5600_CMD_BURN_LOAD_OTP_CONTENT_1 0x11
#define AS5600_CMD_BURN_LOAD_OTP_CONTENT_2 0x10

/*=== Вспомогательные значения ===*/
// Предельное значение регистра CONF_ZMCO
#define AS5600_MAX_VALUE_ZMCO 0x03
// Минимальный угол 18 градусов, примерно 205
#define AS5600_MIN_ANGLE_VALUE_DEC 205
// Ответы стандартного вида успех/ошибка
#define AS5600_DEFAULT_REPORT_ERROR false
#define AS5600_DEFAULT_REPORT_OK    true
// Состояние сторожевого таймера
#define AS5600_WATCHDOG_OFF false
#define AS5600_WATCHDOG_ON  true
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
  AS5600_BURN_REPROT_SENSOR_NOT_CONNECTED
  AS5600_BURN_REPROT_MAGNET_NOT_FOUND,
  AS5600_BURN_REPROT_WRITE_OK,
  AS5600_BURN_REPROT_WRITE_WRONG,
  AS5600_BURN_REPROT_WRITE_OK_WITHOUT_VERIFY,
  AS5600_BURN_REPROT_ZPOS_MPOS_NOT_SET,
  AS5600_BURN_REPROT_ATTEMPTS_ENDED,
  AS5600_BURN_REPROT_ANGLE_VALUE_TOO_SMALL,
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
    TwoWire *__wire; // Объект для использования методов I2C
    int8_t _pin_direction_ = -1; // Контакт микроконтроллера к которому подключен вывод DIR датчика
    AS5600Button _virtual_button; // Структура с параметрами виртуальной кнопки

  protected:
    void AS_SendFirstRegister(uint8_t _reg_addr); // Отправить адрес регистра
	
    uint8_t AS_RequestSingleRegister(void); // Запрос значения регистра размером 1 байт
    uint16_t AS_RequestPairRegisters(void); // Запрос значения регистра размером 2 байта
	
    void AS_WriteOneByte(uint8_t _reg, uint8_t _payload); // Запись одного байта в однобайтовый регистр
    void AS_WriteTwoBytes(uint8_t _low_register, uint8_t _high_register, uint16_t _payload); // Запись двух байтов в двубайтовый регистр
	
  public:
    AS5600(TwoWire *twi); // Конструктор с использованием только интерфейса I2C
    AS5600(TwoWire *twi, int8_t _pin_dir, AS5600DirectionPolarity _def_polar_dir = AS5600_DIRECTION_POLARITY_CLOCKWISE); // Конструктор с использованием интерфейса I2C, контакта DIR, направлением вращения

    void begin(void); // Вызов Wire.begin()
    void setClock(uint32_t freq_hz = AS5600_I2C_CLOCK_400KHZ); // Настройка частоты на 100кГц, 400кГц, 1МГц, или пользовательское значение (по умолчанию 400кГц)
    void end(void); // Вызов Wire.end()
	
    void loadSavedValues(void); // Метод производителя для загрузки значений из памяти в регистры ZPOS, MPOS, MANG, CONF
	
    bool isConnected(void); // Проверка по стандартному алгоритму поиска устройств на шине I2C
	
    /* Виртуальная кнопка */
    /** Настройки **/
    void setButtonMinAGC(byte _btn_min_agc); // Установить новое минимальное значение срабатывания кнопки
    byte getButtonMinAGC(void); // Получить минимальное значение срабатывания кнопки
    void setButtonMaxAGC(byte _btn_max_agc); // Установить новое максимальное значение срабатывания кнопки
    byte getButtonMaxAGC(void); // Получить максимальное значение срабатывания кнопки
    void setButtonDeviation(byte _btn_div); // Установить новое значение отклонения срабатывания кнопки
    byte getButtonDeviation(void); // Получить значение отклонения срабатывания кнопки
    /** События **/
    bool isButtonPressed(void); // Проверка виртуальной кнопки на состояние НАЖАТА
    bool isButtonReleased(void); // Проверка виртуальной кнопки на состояние ОТПУЩЕНА
	
    /* Управление контактом DIR датчика */
    void attachDirectionPin(byte _pin_dir); // Назначить контакт микроконтроллера для управления положительным направлением вращения
    bool setDirection(AS5600DirectionPolarity _direction_polarity); // Установить положительное направление вращения (по/против часовой стрелки)
    bool reverseDirection(void); // Изменить положительное направление вращения на противоположное
    bool getDirection(void); // Получить текущее положительное направление вращения
	
    /* Configuration Registers */
    byte getBurnPositionsCount(void); // Получить количество записей значений в ZPOS и MPOS (с завода ZMCO = 00). 0 - 3
	
    word getZeroPosition(void); // Получить значение начального положения (начальный угол). 0 - 4095
    void setZeroPosition(word _zero_position); // Установить новое начальное положение ZPOS
    bool setZeroPositionVerify(word _zero_position); // Тоже самое, но с подтверждением
    void setZeroPositionViaRawAngle(void); // Установить новое начальное положение ZPOS используя нынешнее положение магнита (getRawAngle)
    bool setZeroPositionViaRawAngleVerify(void); // Тоже самое, но с подтверждением
	
    word getMaxPosition(void); // Получить значение конечного положения (конечный угол). 0 - 4095
    void setMaxPosition(word _max_position); // Установить новое конечное положение MPOS
    bool setMaxPositionVerify(word _max_position); // Тоже самое, но с подтверждением
    void setMaxPositionViaRawAngle(void); // Установить новое начальное положение MPOS используя нынешнее положение магнита (getRawAngle)
    bool setMaxPositionViaRawAngleVerify(void); // Тоже самое, но с подтверждением
	
    word getMaxAngle(void); // Получить значение максимально угла. 0 - 4095
    void setMaxAngle(word _max_angle); // Установить новое значение максимального угла MANG
    bool setMaxAngleVerify(word _max_angle); // Тоже самое, но с подтверждением
    void setMaxAngleViaRawAngle(void); // Установить новое начальное положение MANG используя нынешнее положение магнита (getRawAngle)
    bool setMaxAngleViaRawAngleVerify(void); // Тоже самое, но с подтверждением
	
    word getRawConfigurationValue(void); // Получить "сырые" значения регистра конфигураций CONF. 0 - 4095
    void setRawConfigurationValue(word _confuration_value); // Установить новые "сырые" значения регистра конфигураций CONF
    bool setRawConfigurationValueVerify(word _confuration_value); // Тоже самое, но с подтверждением
    /** Управление Power Mode битами PM **/
    AS5600PowerModes getPowerMode(void); // Получить текущий режим питания
    void setPowerMode(AS5600PowerModes _power_mode); // Установить новый режим питания
    bool setPowerModeVerify(AS5600PowerModes _power_mode); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableNomPowerMode(void); // Включить нормальный режим питания
    bool enableNomPowerModeVerify(void); // Тоже самое, но с подтверждением
    void enableLowPowerMode1(void); // Включить пониженный режим питания 1
    bool enableLowPowerMode1Verify(void); // Тоже самое, но с подтверждением
    void enableLowPowerMode2(void); // Включить пониженный режим питания 2
    bool enableLowPowerMode2Verify(void); // Тоже самое, но с подтверждением
    void enableLowPowerMode3(void); // Включить пониженный режим питания 3
    bool enableLowPowerMode3Verify(void); // Тоже самое, но с подтверждением
    /** Управление Hysteresis битами HYST **/
    AS5600Hysteresis getHysteresis(void); // Получить параметры гистерезиса
    void setHysteresis(AS5600Hysteresis _hysteresis); // Установить новые параметры гистерезиса
    bool setHysteresisVerify(AS5600Hysteresis _hysteresis); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void disableHysteresis(void); // Отключить гистерезис
    bool disableHysteresisVerify(void); // Тоже самое, но с подтверждением
    void enableHysteresis1LSB(void); // Включить гистерезис 1 LSB
    bool enableHysteresis1LSBVerify(void); // Тоже самое, но с подтверждением
    void enableHysteresis2LSB(void); // Включить гистерезис 3 LSB
    bool enableHysteresis2LSBVerify(void); // Тоже самое, но с подтверждением
    void enableHysteresis3LSB(void); // Включить гистерезис 3 LSB
    bool enableHysteresis3LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Output Stage битами OUTS **/
    AS5600OutputStage getOutputStage(void); // Получить режим работы контакта OUT
    void setOutputStage(AS5600OutputStage _output_stage); // Установить режим работы контакта OUT
    bool setOutputStageVerify(AS5600OutputStage _output_stage); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableOutputAnalogFullRange(void); // OUT как аналоговый выход (0 - 100%)
    bool enableOutputAnalogFullRangeVerify(void); // Тоже самое, но с подтверждением
    void enableOutputAnalogReducedRange(void); // OUT как аналоговый выход (10 - 90%)
    bool enableOutputAnalogReducedRangeVerify(void); // Тоже самое, но с подтверждением
    void enableOutputDigitalPWM(void); // OUT как цифровой ШИМ выход
    bool enableOutputDigitalPWMVerify(void); // Тоже самое, но с подтверждением
    /** Управление PWM Frequency битами PWMF **/
    AS5600PWMFrequency getPWMFrequency(void); // Получить значение частоты ШИМ
    void setPWMFrequency(AS5600PWMFrequency _pwm_frequency); // Установить новое значение частоты ШИМ
    bool setPWMFrequencyVerify(AS5600PWMFrequency _pwm_frequency); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enablePWMFrequency115Hz(void); // Включить ШИМ 115Гц
    bool enablePWMFrequency115HzVerify(void); // Тоже самое, но с подтверждением
    void enablePWMFrequency230Hz(void); // Включить ШИМ 230Гц
    bool enablePWMFrequency230HzVerify(void); // Тоже самое, но с подтверждением
    void enablePWMFrequency460Hz(void); // Включить ШИМ 460Гц
    bool enablePWMFrequency460HzVerify(void); // Тоже самое, но с подтверждением
    void enablePWMFrequency920Hz(void); // Включить ШИМ 920Гц
    bool enablePWMFrequency920HzVerify(void); // Тоже самое, но с подтверждением
    /** Управление Slow Filter битами SF **/
    AS5600SlowFilter getSlowFilter(void); // Получить коэффициент медленной фильтрации
    void setSlowFilter(AS5600SlowFilter _slow_filter); // Установить новый коэффициент медленной фильтрации
    bool setSlowFilterVerify(AS5600SlowFilter _slow_filter); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableSlowFilter16x(void); // Включить коэффициент 16х
    bool enableSlowFilter16xVerify(void); // Тоже самое, но с подтверждением
    void enableSlowFilter8x(void); // Включить коэффициент 8х
    bool enableSlowFilter8xVerify(void); // Тоже самое, но с подтверждением
    void enableSlowFilter4x(void); // Включить коэффициент 4х
    bool enableSlowFilter4xVerify(void); // Тоже самое, но с подтверждением
    void enableSlowFilter2x(void); // Включить коэффициент 2х
    bool enableSlowFilter2xVerify(void); // Тоже самое, но с подтверждением
    /** Управление Fast Filter Threshold битами FTH **/
    AS5600FastFilterThreshold getFastFilterThreshold(void); // Получить порог быстрой фильтрации
    void setFastFilterThreshold(AS5600FastFilterThreshold _fast_filter_thredhold); // Установить порог быстрой фильтрации
    bool setFastFilterThresholdVerify(AS5600FastFilterThreshold _fast_filter_thredhold); // Тоже самое, но с подтверждением
    // Отдельные режимы
    void enableSlowFilterOnly(void); // Включить только медленную фильтрацию
    bool enableSlowFilterOnlyVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold6LSB(void); // Включить быструю фильтрацию 6 LSB
    bool enableFastFilterThreshold6LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold7LSB(void); // Включить быструю фильтрацию 7 LSB
    bool enableFastFilterThreshold7LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold9LSB(void); // Включить быструю фильтрацию 9 LSB
    bool enableFastFilterThreshold9LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold18LSB(void); // Включить быструю фильтрацию 18 LSB
    bool enableFastFilterThreshold18LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold21LSB(void); // Включить быструю фильтрацию 21 LSB
    bool enableFastFilterThreshold21LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold24LSB(void); // Включить быструю фильтрацию 24 LSB
    bool enableFastFilterThreshold24LSBVerify(void); // Тоже самое, но с подтверждением
    void enableFastFilterThreshold10LSB(void); // Включить быструю фильтрацию 10 LSB
    bool enableFastFilterThreshold10LSBVerify(void); // Тоже самое, но с подтверждением
    /** Управление Watchdog битом WD **/
    bool isWatchdog(void); // Определить состояние сторожевого таймера
    // Отдельные способы
    void enableWatchdog(void); // Включить сторожевой таймер
    bool enableWatchdogVerify(void); // Тоже самое, но с подтверждением
    void disableWatchdog(void); // Выключить сторожевой таймер
    bool disableWatchdogVerify(void); // Тоже самое, но с подтверждением
    
    /* Output Registers */
    word getRawAngle(void); // Получить угол в чистом виде. 0 - 4095
    float getDegreesAngle(void); // Получить угол в градусах. 0.00 - 360.00. Основан на значениях от getRawAngle
    float getRadiansAngle(void); // Получить угол в радианах 0.00 - 6.29. Основан на значениях от getRawAngle
    
    word getScaledAngle(void); // Получить масштабированный угол с учетом ZPOS, MPOS или MANG. 0 - 4095
	
    /* Status Registers */
    AS5600StatusReports getStatus(void); // Получить значение регистра STATUS
    bool isMagnetDetected(void); // Определить наличие магнита MD
    bool isMagnetTooWeak(void); // Определить очень слабый магнит ML
    bool isMagnetTooStrong(void); // Определить очень сильный магнит MH
	
    byte getAutomaticGainControl(void); // Получить значение автоусиления. При VCC = 5В -> 0 - 255, при VCC = 3.3В -> 0 - 128
	
    word getMagnitude(void); // Получить значение магнитуды. 0 - 4095
    
    /* Burn Commands */
    AS5600BurnReports burnZeroAndMaxPositions(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда ZPOS, MPOS. CMD_BURN_ANGLE [3 РАЗА МАКСИМУМ!]
    AS5600BurnReports burnMaxAngleAndConfigurationValue(AS5600SpecialVerifyFlags _use_special_verify = AS5600_FLAG_SPECIAL_VERIFY_ENABLE); // Записать навсегда MANG, CONF. CMD_BURN_SETTINGS [1 РАЗ МАКСИМУМ!]
};
