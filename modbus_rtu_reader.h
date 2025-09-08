//
// Created by Андрей Водолацкий on 07.09.2025.
//

#ifndef MODBUS_RTU_READER_H
#define MODBUS_RTU_READER_H

#include <string>
#include <vector>
#include <cstdint>
#include <termios.h> // Для speed_t


// Modbus константы
#define MODBUS_SLAVE_ADDRESS 0x01
#define MODBUS_BROADCAST_ADDRESS 0x00
#define MAX_FRAME_SIZE 256
#define MAX_REGISTERS 100

// Команды Modbus
// Чтение дискретных выходов (Coils)	0x01
// Чтение дискретных входов (Discrete Inputs)	0x02
// Чтение аналоговых выходов (Holding Registers)	0x03
// Чтение аналоговых входов (Input Registers)	0x04
// Запись одного дискретного выхода (Coil)	0x05
// Запись одного регистра (Holding Register)	0x06
// Запись нескольких дискретных выходов (Coils)	0x0F
// Запись нескольких регистров (Holding Registers)	0x10
#define READ_COILS 0x01
#define READ_DISCRETE_INPUTS 0x02
#define READ_HOLDING_REGISTERS 0x03
#define READ_INPUT_REGISTERS 0x04
#define WRITE_SINGLE_COIL 0x05
#define WRITE_SINGLE_REGISTER 0x06
#define WRITE_MULTIPLE_COILS 0x0F
#define WRITE_MULTIPLE_REGISTERS 0x10

// Коды ошибок
#define ILLEGAL_FUNCTION 0x01
#define ILLEGAL_DATA_ADDRESS 0x02
#define ILLEGAL_DATA_VALUE 0x03
#define SLAVE_DEVICE_FAILURE 0x04

// Структура для хранения регистров
typedef struct {
    uint16_t holding_registers[MAX_REGISTERS];
    uint16_t input_registers[MAX_REGISTERS];
    uint8_t coils[MAX_REGISTERS];
    uint8_t discrete_inputs[MAX_REGISTERS];
} modbus_registers_t;

// Глобальные переменные
modbus_registers_t registers;
uint8_t rx_buffer[MAX_FRAME_SIZE];
std::vector<uint8_t> tx_buffer;
uint8_t rx_index = 0;
uint8_t frame_received = 0;


// Функция вычисления CRC16 Modbus
// uint16_t crc16_modbus(const uint8_t* data, size_t length);

// Класс для чтения и разбора Modbus RTU пакетов
class ModbusRTUReader {
public:
    // Конструктор: device - имя порта, baudRate - скорость передачи
    ModbusRTUReader(const std::string &device, speed_t baudRate);

    // Деструктор: закрывает порт
    ~ModbusRTUReader();

    // Запуск цикла чтения и разбора пакетов
    void readLoop();

private:
    int fd; // Дескриптор порта

    void configurePort(speed_t baudRate) const;

    void createErrorResponse(uint8_t function_code, uint8_t error_code);

    void checkRequestADU(std::vector<uint8_t> &packet);

    static void printHEXPacket(std::vector<uint8_t> &packet);

    void handleReadCoils(std::vector<uint8_t> &frame);

    void handleReadHoldingRegisters(std::vector<uint8_t> &frame);

    void handleWriteSingleRegister(std::vector<uint8_t> &frame);

    void handleWriteMultipleRegisters(std::vector<uint8_t> &frame);

    void sendResponseToMaster(std::vector<uint8_t> &frame) const;

    void processRequestADU(std::vector<uint8_t> &frame);
};

#endif // MODBUS_RTU_READER_H
