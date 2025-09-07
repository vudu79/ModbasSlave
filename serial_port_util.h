//
// Created by Андрей Водолацкий on 07.09.2025.
//

#ifndef UNTITLED_SERIAL_PORT_UTIL_H
#define UNTITLED_SERIAL_PORT_UTIL_H

#include <cstdint>
#include <unistd.h> // для ssize_t

int configure_port(int fd);
int configure_port_common(int fd);
void print_buffer(const char *prefix, const uint8_t *buf, size_t len);
void print_buffer_common(const char *prefix, const char *buf, ssize_t len);
// uint16_t crc16_modbus(const uint8_t *buf, size_t len);

#endif //UNTITLED_SERIAL_PORT_UTIL_H