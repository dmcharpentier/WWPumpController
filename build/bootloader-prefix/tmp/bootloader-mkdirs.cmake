# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/Dylan/esp/v5.1.1/esp-idf/components/bootloader/subproject"
  "C:/Coding/Code/ESP32/WWPumpController/build/bootloader"
  "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix"
  "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix/tmp"
  "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix/src"
  "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Coding/Code/ESP32/WWPumpController/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
