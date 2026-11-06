#!/bin/sh
# Модульные тесты прошивки на ПК (Linux, WSL, macOS, MSYS2): протокол и логика без платы.
cd "$(dirname "$0")" || exit 1
g++ -std=c++98 -Wall -Wextra -pedantic -I../pouk_mcu test_host.cpp \
    ../pouk_mcu/pouk_proto.cpp ../pouk_mcu/mcu_logic.cpp -o test_host && ./test_host
