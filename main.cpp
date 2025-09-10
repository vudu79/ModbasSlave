
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include "modbus_rtu_reader.h"


int main() {

    try {
        ModbusRTUReader reader("/dev/ttys002", B9600);
        reader.readLoop();
    }
    catch (const std::exception& ex) {
        std::cerr << "Ошибка: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
