# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/work/PT01/code/git_pt01/ESP_IDF/components/bootloader/subproject"
  "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader"
  "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix"
  "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix/tmp"
  "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix/src/bootloader-stamp"
  "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix/src"
  "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/work/PT01/code/git_pt01/ESP_IDF/examples/mpu6050/sample_project/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
