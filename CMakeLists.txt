cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Só main e suas dependências: o nó não usa WiFi/BLE, build e binário menores
idf_build_set_property(MINIMAL_BUILD ON)
project(weather_station_tcc)
