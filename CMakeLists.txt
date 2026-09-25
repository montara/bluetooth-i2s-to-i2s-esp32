# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
	message(FATAL_ERROR "IDF_PATH is not set. Run the ESP-IDF export script before configuring this project.")
endif()

set(IDF_PROJECT_CMAKE "$ENV{IDF_PATH}/tools/cmake/project.cmake")
if(NOT EXISTS "${IDF_PROJECT_CMAKE}")
	message(FATAL_ERROR "ESP-IDF project.cmake was not found at ${IDF_PROJECT_CMAKE}.")
endif()

include("${IDF_PROJECT_CMAKE}")
project(bt_i2s_dac)
