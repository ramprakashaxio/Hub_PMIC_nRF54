# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2021 Maxim Integrated Products, Inc.

if(CONFIG_PMIC_MAX77658)

zephyr_library_named(pmic_max77658)

zephyr_library_sources(
    max77658.c
    bsp.c
)

if(CONFIG_PMIC_MAX77658_PM)
    zephyr_library_sources(max77658_pm.c)
endif()

if(CONFIG_PMIC_MAX77658_FG)
    zephyr_library_sources(max77658_fg.c)
endif()

zephyr_library_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link this library to the app
zephyr_library_link_libraries(app)

endif()
