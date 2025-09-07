//
// Created by Андрей Водолацкий on 07.09.2025.
//

#ifndef MODBUS_RTU_READER_H
#define MODBUS_RTU_READER_H

#include <string>
#include <vector>
#include <cstdint>
#include <termios.h> // Для speed_t

// Функция вычисления CRC16 Modbus
// uint16_t crc16_modbus(const uint8_t* data, size_t length);

// Класс для чтения и разбора Modbus RTU пакетов
class ModbusRTUReader {
public:
    // Конструктор: device - имя порта, baudRate - скорость передачи
    ModbusRTUReader(const std::string& device, speed_t baudRate);

    // Деструктор: закрывает порт
    ~ModbusRTUReader();

    // Запуск цикла чтения и разбора пакетов
    void readLoop() const;

private:
    int fd; // Дескриптор порта

    // Настройка параметров порта
    void configurePort(speed_t baudRate) const;

    // Обработка завершённого пакета
    static void processPacket(const std::vector<uint8_t>& packet);
};

#endif // MODBUS_RTU_READER_H