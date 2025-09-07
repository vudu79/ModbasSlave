cmake_minimum_required(VERSION 4.0)
project(ModbusSlave)

set(CMAKE_CXX_STANDARD 20)

add_executable(ModbusSlave main.cpp
        serial_port_util.cpp
        modbus_rtu_reader.cpp)
