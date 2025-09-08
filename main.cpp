
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include "modbus_rtu_reader.h"

#define BUF_SIZE 256

// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
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



    // // Открываем порты
    // int fd1 = open("/dev/ttys001", O_RDWR | O_NOCTTY);
    // if (fd1 < 0) {
    //     perror("open");
    //     return 1;
    // }
    // if (configure_port(fd1) < 0) {
    //     return 1;
    // }
    //
    // ssize_t count = 0;
    // uint8_t frame[255];
    // while (1) {
    //     uint8_t response[1];
    //     ssize_t r = read(fd1, response, 1);
    //     if (r < 0) {
    //         perror("read");
    //         break;
    //     }
    //     if (r == 0 && frame[0] != 0) {
    //         print_buffer("Получен фрейм: ", frame, count);
    //         std::memset(frame, 0, sizeof(frame));
    //         count = 0;
    //     } else if (r == 0 && frame[0] == 0) {
    //         // printf("%ld Нет данных в порту 2\n", count);
    //     } else {
    //         frame[count++] = *response;
    //         // print_buffer("Получен ответ из порта:", response, r);
    //     }
    // }
    //
    // close(fd1);
    // // close(fd2);
    // return 0;
// }
