# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr COMPONENTS unittest REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(led_strip)

add_library(mocks STATIC src/mocks/gpio.c)
target_include_directories(mocks PUBLIC src/mocks)
target_link_libraries(mocks PRIVATE test_interface)

target_sources(testbinary PRIVATE src/main.c)
target_link_libraries(testbinary PUBLIC mocks)
