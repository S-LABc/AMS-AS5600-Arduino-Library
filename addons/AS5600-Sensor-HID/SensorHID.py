'''
SensorHID.py

Скрипт визуализации принятых данных через  USB HID
от датчика AS5600 или AS5600L. Этот скрипт часть
библиотеки AMS-AS5600-Arduino-Library.
Скетч Arduino STM32/SensorUSBHID_STM32.ino

Требуется pywinusb -> pip install pywinusb
Проверялся на Python 3.10.8

Документация к датчику https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf

Контакты
GitHub: https://github.com/S-LABc
Gmail: romansklyar15@gmail.com 

Copyright (C) 2022. v1.0 / Скляр Роман S-LAB
'''

from pywinusb import hid # модуль для работы с USB HID под Windows
from time import sleep # модуль для паузы
import math # модуль для константы pi

TX_SIZE = 2 # количество передаваемых байт из скрипта через USB по протоколу HID, передается всегда + 1 байт - Report ID

DEV_VID = 0x1EAF # Vendor ID устройства (связан со скетчем Ардуино)
DEV_PID = 0x0024 # Product ID устройства (связан со скетчем Ардуино)

REPORT_ID = 0x00 # Report ID со значением 0, так требует библиотека USBComposite_stm32f1 https://github.com/arpruss/USBComposite_stm32f1
UNIQ_KEY = 0x15 # уникальный ключ для проверки пакета (связан со скетчем Ардуино)

PAUSE_TIME = 1 # время паузы отправления запросов на получение данных в секундах

send_buffer = [i for i in range(TX_SIZE)] # создаем массив для отправки данных, 0 байт нужен как ключ, 1 байт нужен как Report ID
send_buffer[0] = REPORT_ID # устанавливаем 0 элемент массива равный REPORT_ID
send_buffer[1] = UNIQ_KEY # устанавливаем 1 элемент массива равный UNIQ_KEY

# метод обработки и вывода принятых по USB HID данных в терминал
def receive_handler(data):
    print("================================")
    print("Состояние датчика: ", end = '')
    if data[1] == 0:
        print("НЕ ОБНАРУЖЕН!")
    if data[1] == 1:
        print("ОБНАРУЖЕН!")
    else:
        print("ОШИБКА")

    start_pos = data[2] << 8 | data[3]
    print("Начальное положение: ", start_pos)
    
    end_pos = data[4] << 8 | data[5]
    print("Конечное положение: ", end_pos)

    max_ang = data[6] << 8 | data[7]
    print("Максимальный угол: ", max_ang)

    print("Режим питания: ", end = '')
    if data[8] == 0:
        print("Нормальный")
    elif data[8] == 1:
        print("Энергосбережение 1")
    elif data[8] == 2:
        print("Энергосбережение 2")
    elif data[8] == 3:
        print("Энергосбережение 3")
    else:
        print("ОШИБКА")

    print("Гистерезис: ", end = '')
    if data[9] == 0:
        print("Выключен")
    elif data[9] == 1:
        print("1 LSB")
    elif data[9] == 2:
        print("2 LSB")
    elif data[9] == 3:
        print("3 LSB")
    else:
        print("ОШИБКА")

    print("Режим контакта OUT: ", end = '')
    if data[10] == 0:
        print("Аналоговый (0 - 100%)")
    elif data[10] == 1:
        print("Аналоговый (10 - 90%)")
    elif data[10] == 2:
        print("Цифровой ШИМ")
    else:
        print("ОШИБКА")

    print("Несущая частота ШИМ: ", end = '')
    if data[11] == 0:
        print("115 Гц")
    elif data[11] == 1:
        print("230 Гц")
    elif data[11] == 2:
        print("460 Гц")
    elif data[11] == 3:
        print("920 Гц")
    else:
        print("ОШИБКА")

    print("Коэф. медленной фильтрации: ", end = '')
    if data[12] == 0:
        print("16х")
    elif data[12] == 1:
        print("8х")
    elif data[12] == 2:
        print("4х")
    elif data[12] == 3:
        print("2х")
    else:
        print("ОШИБКА")

    print("Порог быстрой фильтрации: ", end = '')
    if data[13] == 0:
        print("Включена медленная фильтрация")
    elif data[13] == 1:
        print("6 LSB")
    elif data[13] == 2:
        print("7 LSB")
    elif data[13] == 3:
        print("9 LSB")
    elif data[13] == 4:
        print("18 LSB")
    elif data[13] == 5:
        print("21 LSB")
    elif data[13] == 6:
        print("24 LSB")
    elif data[13] == 7:
        print("10 LSB")
    else:
        print("ОШИБКА")

    print("Сторожевой таймер: ", end = '')
    if data[14] == 0:
        print("Выключен")
    elif data[14] == 1:
        print("Включен")
    else:
        print("ОШИБКА")
    
    raw_ang = data[15] << 8 | data[16]
    print("Угол в сыром виде: ", raw_ang)

    deg_ang = raw_ang * 360 / 4096
    print("Угол в градусах: ", deg_ang)

    rad_ang = deg_ang * math.pi / 180
    print("Угол в радианах: ", rad_ang)

    scl_ang = data[17] << 8 | data[18]
    print("Масштабированный угол: ", scl_ang)

    print("Состояние магнита: ", end = '')
    if data[19] == 0:
        print("Не обнаружен")
    elif data[19] == 2:
        print("Не обнаружен и слишком слабый")
    elif data[19] == 4:
        print("Обнаружен и нормальный")
    elif data[19] == 5:
        print("Обнаружен и слишком сильный")
    elif data[19] == 6:
        print("Обнаружен и слишком слабый")
    else:
        print("ОШИБКА")

    print("Автоусиление: ", data[20])

    mag = data[21] << 8 | data[22]
    print("Магнитуда: ", mag)

    print("Счетчик ZMCO: ", data[23])

    print("================================")
    print()

    return None 

hid_list = hid.HidDeviceFilter(vendor_id = DEV_VID, product_id = DEV_PID) # создаем фильтр устройств HID в ОС
devices_list = hid_list.get_devices() # получаем устройства с нужными VID и PID
device = devices_list[0] # выбираем самое первое из доступных
device.set_raw_data_handler(receive_handler) # устанавливаем обработчик на прием данных (что выполнять когда пришли данные от устройства по USB HID)
device.open() # открываем канал связи по USB HID с устройством

while True:
    for out_report in device.find_output_reports():
        out_report.set_raw_data(send_buffer) # указываем массив с данными для отправки в устройство
        out_report.send() # отправляем данные в устройство
        sleep(PAUSE_TIME) # пауза для удобства чтения данных в терминале
