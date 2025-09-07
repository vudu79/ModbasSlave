#include "serial_port_util.h"

#include <cstdint>
#include <stdio.h>
#include <termios.h>
#include <ctype.h>

// настрока порта обшее
int configure_port_common(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}


// Функция для настройки порта modbus (пример, нужно адаптировать под вашу систему)
int configure_port(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8 бит
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // no parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}

// печать буфера общее
void print_buffer_common(const char *prefix, const char *buf, ssize_t len) {
    printf("%s %zd bytes: ", prefix, len);
    for (ssize_t i = 0; i < len; i++) {
        printf("%02X ", (unsigned char)buf[i]);
    }
    printf(" | ");
    for (ssize_t i = 0; i < len; i++) {
        char c = buf[i];
        printf("%c", isprint((unsigned char)c) ? c : '.');
    }
    printf("\n");
}

// Функция для печати буфера в hex и ASCII modbus
void print_buffer(const char *prefix, const uint8_t *buf, size_t len) {
    printf("%s %zu bytes: ", prefix, len);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", buf[i]);
    }
    printf(" | ");
    for (size_t i = 0; i < len; i++) {
        if (buf[i] >= 32 && buf[i] <= 126)
            printf("%c", buf[i]);
        else
            printf(".");
    }
    printf("\n");
}

// uint16_t crc16_modbus(const uint8_t *buf, size_t len) {
//     uint16_t crc = 0xFFFF;
//     for (size_t pos = 0; pos < len; pos++) {
//         crc ^= buf[pos];
//         for (int i = 0; i < 8; i++) {
//             if (crc & 1) crc = (crc >> 1) ^ 0xA001;
//             else crc >>= 1;
//         }
//     }
//     return crc;
// }