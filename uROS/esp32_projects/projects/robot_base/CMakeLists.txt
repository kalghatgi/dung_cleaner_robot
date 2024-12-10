cmake_minimum_required(VERSION 3.5)

list(APPEND EXTRA_COMPONENT_DIRS "../../components")

set(PROJECT_VER "1.4")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robot_base)

