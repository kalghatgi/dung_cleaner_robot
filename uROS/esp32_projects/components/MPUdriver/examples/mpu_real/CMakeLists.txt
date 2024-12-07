cmake_minimum_required(VERSION 3.5)

list(APPEND EXTRA_COMPONENT_DIRS "../../../MPUdriver" "../../../I2Cbus")

set(PROJECT_VER "1.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(mpu_real)

