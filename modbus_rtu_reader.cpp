#include "modbus_rtu_reader.h"

#include <iostream>     // Для вывода в консоль
#include <vector>       // Для динамического массива байт (буфера)
#include <chrono>       // Для измерения времени (таймаут)
#include <fcntl.h>      // Для работы с файловыми дескрипторами (open)
#include <unistd.h>     // Для read, close, usleep
#include <termios.h>    // Для настройки последовательного порта
#include <cstring>      // Для strerror
#include <stdexcept>    // Для исключений

// Глобальные переменные
modbus_registers_t registers;
std::vector<uint8_t> tx_buffer(MAX_FRAME_SIZE);
uint8_t rx_index = 0;
uint8_t frame_received = 0;

// Функция вычисления CRC16 Modbus
uint16_t crc16_modbus(std::vector<uint8_t> &frame, size_t length) {
    uint16_t crc = 0xFFFF; // Инициализация CRC стартовым значением
    for (size_t pos = 0; pos < length; pos++) {
        crc ^= static_cast<uint16_t>(frame[pos]); // XOR с очередным байтом
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
} // Возвращаем вычисленное CRC


// Класс для чтения и разбора Modbus RTU пакетов
ModbusRTUReader::ModbusRTUReader(const std::string &device, speed_t baudRate) : fd(-1)
// Инициализируем дескриптор невалидным значением
{
    fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC); // Открываем порт
    if (fd < 0) {
        // Если не удалось открыть, выбрасываем исключение с сообщением
        throw std::runtime_error("Не удалось открыть порт " + device + ": " + strerror(errno));
    }
    configurePort(baudRate); // Настраиваем параметры порта
}

// Деструктор: закрывает порт при уничтожении объекта
ModbusRTUReader::~ModbusRTUReader() {
    if (fd >= 0) close(fd);
}

// Основной цикл чтения данных из порта
void ModbusRTUReader::readLoop() {
    std::vector<uint8_t> buffer; // Буфер для накопления байт пакета
    buffer.clear();
    using clock = std::chrono::steady_clock; // Тип часов для измерения времени
    auto lastByteTime = clock::now(); // Время получения последнего байта

    // Таймаут 3.5 (по протоколу) символа в микросекундах (используем 9600 бод)
    // 1 символ ~ 11 бит / 9600 бод = ~1.15 мс
    // 3.5 символа ~ 4 мс = 4000 мкс
    constexpr auto timeout = std::chrono::microseconds(4000);

    while (true) {
        uint8_t byte;
        ssize_t n = read(fd, &byte, 1); // Читаем 1 байт из порта

        if (n < 0) {
            std::cerr << "Ошибка чтения: " << strerror(errno) << std::endl;
            continue; // При ошибке чтения продолжаем попытки
        }

        if (n == 0 && buffer.size() != 8) {
            // Нет данных, ждем
            usleep(1000); // 1 мс
            continue;
        }

        auto now = clock::now(); // Текущее время

        if (!buffer.empty()) {
            // Если буфер не пуст, проверяем паузу между байтами
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(now - lastByteTime);
            if (diff > timeout) {
                // Если пауза больше 3.5 символов — считаем пакет завершённым
                checkRequestADU(buffer); // Обрабатываем накопленный пакет
                buffer.clear(); // Очищаем буфер для нового пакета
            }
        }
        // Добавляем байт в буфер
        if (n == 1) {
            buffer.push_back(byte);
            lastByteTime = now; // Обновляем время последнего байта
        }
    }
}

// Обработка завершённого пакета
void ModbusRTUReader::checkRequestADU(std::vector<uint8_t> &packet) {
    // не наш адрес или не широковещательный - не берем
    if (packet[0] != MODBUS_SLAVE_ADDRESS && packet[0] != MODBUS_BROADCAST_ADDRESS) {
        return; // Не наш адрес
    }

    if (packet.size() < 4) {
        // Минимальный размер пакета: адрес(1) + функция(1) + CRC(2)
        std::cerr << "Пакет слишком короткий, игнорируем" << std::endl;
        return;
    }

    // Вычисляем CRC по всем байтам, кроме последних двух (CRC в конце)
    uint16_t crcCalc = crc16_modbus(packet, packet.size() - 2);
    // Извлекаем CRC из пакета (младший байт + старший байт)
    uint16_t crcPacket = packet[packet.size() - 2] | (packet[packet.size() - 1] << 8);

    if (crcCalc != crcPacket) {
        std::cerr << "Ошибка CRC, пакет отброшен" << std::endl;
        return;
    }

    // Если CRC верен — выводим пакет в консоль
    std::cout << "Получен корректный пакет ADU: ";
    for (auto b: packet) {
        printf("%02X ", b);
    }
    std::cout << std::endl;

    // Разбираем полученный ADU, реагируем на команду
    processRequestADU(packet);
}

void ModbusRTUReader::printHEXPacket(std::vector<uint8_t> &packet) {
    uint8_t slaveNumber = packet[0];
    uint8_t functionCode = packet[1];
    if (functionCode  & 0x80) {
        printf("Пакет с кодом ошибки на команду № %d от устройства № %d сформирован: ", functionCode, slaveNumber);
    }
    printf("Ответный пакет на команду № %d от устройства № %d сформирован: ", functionCode, slaveNumber);
    for (auto b: packet) {
        printf("%02X ", b);
    }
    std::cout << std::endl;
}

// Настройка параметров последовательного порта
void ModbusRTUReader::configurePort(speed_t baudRate) const {
    termios tty{}; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Ошибка пtcsetattr: " + std::string(strerror(errno)));
    }

    cfsetospeed(&tty, baudRate); // Установка скорости передачи (выход)
    cfsetispeed(&tty, baudRate); // Установка скорости передачи (вход)

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 бит данных
    tty.c_iflag &= ~IGNBRK; // Не игнорировать BREAK
    tty.c_lflag = 0; // Режим "raw" (без обработки)
    tty.c_oflag = 0; // Вывод без обработки
    tty.c_cc[VMIN] = 0; // Минимум байт для чтения
    tty.c_cc[VTIME] = 0; // Таймаут чтения

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Отключаем управление потоком XON/XOFF
    tty.c_cflag |= (CLOCAL | CREAD); // Включаем прием и локальный режим
    tty.c_cflag &= ~(PARENB | PARODD); // Без проверки четности
    tty.c_cflag &= ~CSTOPB; // 1 стоп-бит
    tty.c_cflag &= ~CRTSCTS; // Отключаем аппаратное управление потоком

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        throw std::runtime_error("Ошибка tcsetattr: " + std::string(strerror(errno)));
    }
}

// Создание ответа с ошибкой
void ModbusRTUReader::createErrorResponse(uint8_t function_code, uint8_t error_code) const {
    tx_buffer[0] = MODBUS_SLAVE_ADDRESS;
    tx_buffer[1] = function_code | 0x80; // Установка бита ошибки
    tx_buffer[2] = error_code;

    uint16_t crc = crc16_modbus(tx_buffer, 3);
    tx_buffer[3] = crc & 0xFF;
    tx_buffer[4] = (crc >> 8) & 0xFF;

    sendResponseToMaster(tx_buffer, 4);
}

// Отправка ответа на команду мастера в порт
void ModbusRTUReader::sendResponseToMaster(std::vector<uint8_t> &frame, int len) const {
    std::vector<uint8_t> data(len);
    std::copy(frame.begin(), frame.end(), data.begin());

    printHEXPacket(data);

    ssize_t w = write(fd, data.data(), len);
    printf("Размер пакета - %d. Отправлено - %ld\n", len, w);


}


// Обработка команды Read Coils (0x01) Чтение дискретных выходов
void ModbusRTUReader::handleReadCoils(std::vector<uint8_t> &frame) const {
    uint16_t start_address = (frame[2] << 8) | frame[3];
    uint16_t quantity = (frame[4] << 8) | frame[5];

    if (start_address + quantity > MAX_REGISTERS) {

        createErrorResponse(READ_COILS, ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t byte_count = (quantity + 7) / 8;
    tx_buffer[0] = MODBUS_SLAVE_ADDRESS;
    tx_buffer[1] = READ_COILS;
    tx_buffer[2] = byte_count;

    uint8_t i, j;
    for (i = 0; i < byte_count; i++) {
        tx_buffer[3 + i] = 0;
        for (j = 0; j < 8 && (i * 8 + j) < quantity; j++) {
            if (registers.coils[start_address + i * 8 + j]) {
                tx_buffer[3 + i] |= (1 << j);
            }
        }
    }

    uint16_t crc = crc16_modbus(tx_buffer, 3 + byte_count);
    tx_buffer[3 + byte_count] = crc & 0xFF;
    tx_buffer[4 + byte_count] = (crc >> 8) & 0xFF;
    printf("crc - %x\n", tx_buffer[3 + byte_count]);
    printf("crc - %x\n", tx_buffer[4 + byte_count]);

    sendResponseToMaster(tx_buffer, 4 + byte_count +1);
}

// Обработка команды Read Holding Registers (0x03) Чтение аналоговых выходов
void ModbusRTUReader::handleReadHoldingRegisters(std::vector<uint8_t> &frame) const {
    const uint16_t start_address = (frame[2] << 8) | frame[3];
    const uint16_t quantity = (frame[4] << 8) | frame[5];

    if (start_address + quantity > MAX_REGISTERS) {
        createErrorResponse(READ_HOLDING_REGISTERS, ILLEGAL_DATA_ADDRESS);
        return;
    }

    tx_buffer[0] = MODBUS_SLAVE_ADDRESS;
    tx_buffer[1] = READ_HOLDING_REGISTERS;
    tx_buffer[2] = quantity * 2;

    uint8_t i;
    for (i = 0; i < quantity; i++) {
        tx_buffer[3 + i * 2] = (registers.holding_registers[start_address + i] >> 8) & 0xFF;
        tx_buffer[4 + i * 2] = registers.holding_registers[start_address + i] & 0xFF;
    }

    uint16_t crc = crc16_modbus(tx_buffer, 3 + quantity * 2);
    tx_buffer[3 + quantity * 2] = crc & 0xFF;
    tx_buffer[4 + quantity * 2] = (crc >> 8) & 0xFF;
    u_int8_t rrr = 4 + quantity * 2;

    sendResponseToMaster(tx_buffer, (5 + quantity * 2));
}

// Обработка команды Write Single Register (0x06)
void ModbusRTUReader::handleWriteSingleRegister(std::vector<uint8_t> &frame) {
    uint16_t address = (frame[2] << 8) | frame[3];
    uint16_t value = (frame[4] << 8) | frame[5];

    if (address >= MAX_REGISTERS) {
        createErrorResponse(WRITE_SINGLE_REGISTER, ILLEGAL_DATA_ADDRESS);
        return;
    }

    registers.holding_registers[address] = value;

    // Эхо ответа
    memcpy(tx_buffer.data(), frame.data(), 8);

    uint16_t crc = crc16_modbus(tx_buffer, 6);
    tx_buffer[6] = crc & 0xFF;
    tx_buffer[7] = (crc >> 8) & 0xFF;

    sendResponseToMaster(tx_buffer, 7 +1);
}

// Обработка команды Write Multiple Registers (0x10)
void ModbusRTUReader::handleWriteMultipleRegisters(std::vector<uint8_t> &frame) {
    uint16_t start_address = (frame[2] << 8) | frame[3];
    uint16_t quantity = (frame[4] << 8) | frame[5];
    uint8_t byte_count = frame[6];

    if (start_address + quantity > MAX_REGISTERS || byte_count != quantity * 2) {
        createErrorResponse(WRITE_MULTIPLE_REGISTERS, ILLEGAL_DATA_ADDRESS);
        return;
    }

    uint8_t i;
    for (i = 0; i < quantity; i++) {
        registers.holding_registers[start_address + i] =
                (frame[8 + i * 2] << 8) | frame[9 + i * 2];
    }

    // Ответ
    tx_buffer[0] = MODBUS_SLAVE_ADDRESS;
    tx_buffer[1] = WRITE_MULTIPLE_REGISTERS;
    tx_buffer[2] = (start_address >> 8) & 0xFF;
    tx_buffer[3] = start_address & 0xFF;
    tx_buffer[4] = (quantity >> 8) & 0xFF;
    tx_buffer[5] = quantity & 0xFF;

    uint16_t crc = crc16_modbus(tx_buffer, 6);
    tx_buffer[6] = crc & 0xFF;
    tx_buffer[7] = (crc >> 8) & 0xFF;

    sendResponseToMaster(tx_buffer, 7 +1);
}

// Обработка Modbus кадра
void ModbusRTUReader::processRequestADU(std::vector<uint8_t> &frame) {
    uint8_t function_code = frame[1];
    switch (function_code) {
        case READ_COILS:
            handleReadCoils(frame);
            break;
        case READ_DISCRETE_INPUTS:
            // Аналогично READ_COILS
            break;
        case READ_HOLDING_REGISTERS:
            handleReadHoldingRegisters(frame);
            break;
        case READ_INPUT_REGISTERS:
            // Аналогично READ_HOLDING_REGISTERS
            break;
        case WRITE_SINGLE_COIL:
            // Аналогично WRITE_SINGLE_REGISTER
            break;
        case WRITE_SINGLE_REGISTER:
            handleWriteSingleRegister(frame);
            break;
        case WRITE_MULTIPLE_COILS:
            // Аналогично WRITE_MULTIPLE_REGISTERS
            break;
        case WRITE_MULTIPLE_REGISTERS:
            handleWriteMultipleRegisters(frame);
            break;
        default:
            createErrorResponse(function_code, ILLEGAL_FUNCTION);
            break;
    }
}
