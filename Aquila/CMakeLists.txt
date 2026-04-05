# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS 
    "components/general_components/IMUs"
    "components/general_components/baros"
    "components/general_components/compass"
    "components/general_components/IOs"
    "components/general_components/optical_flow"
    "components/general_components/power_monitors"
    "components/general_components/lidars"
    "components/general_components/LEDs"
    "components/general_components/flash_memory"
    "components/general_components/ahrs_algorithms"
    "components/general_components/ahrs_algorithms/fusion"
    "components/general_components/ahrs_algorithms/vqf"
    "components/general_components/ahrs_algorithms/Madgwick"
    "components/general_components/mavlink"
    "components/general_components/ESP_operations"
    "components/general_components/math_operations"
    "components/general_components/pid"
    "components/general_components/esc_control"
    "components/general_components/esc_control/pwm"
    "components/general_components/esc_control/dshot"    

)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Aquila LANGUAGES C CXX)




