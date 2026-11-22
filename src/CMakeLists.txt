# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
  SRCS "main.c"
  SRCS "bluetooth_spp.c"
  SRCS "mpu6050_imu.c"
  SRCS "link_rx.c"
  INCLUDE_DIRS "." REQUIRES esp_driver_i2c)
