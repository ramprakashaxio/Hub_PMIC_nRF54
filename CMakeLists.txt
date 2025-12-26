# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pmic_2512)

# Application sources
target_sources(app PRIVATE 
    src/main.c
    src/pmic_gpio.c
    pmic/max77658.c
    pmic/bsp.c
)

# Conditionally add PMIC module sources
if(CONFIG_PMIC_MAX77658_PM)
    target_sources(app PRIVATE pmic/max77658_pm.c)
endif()

if(CONFIG_PMIC_MAX77658_FG)
    target_sources(app PRIVATE pmic/max77658_fg.c)
endif()

# Include directories
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/pmic
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
