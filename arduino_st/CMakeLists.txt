
cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(PROJECT_NAME arduino_st)

#https://github.com/queezythegreat/arduino-cmake#compiler-and-linker-flags
#should be set prior project
#ARDUINO_C_FLAGS contain here code size optimization
set(ARDUINO_C_FLAGS      " -Os -ffunction-sections -fdata-sections -Wl,--gc-sections -fdce")
set(ARDUINO_CXX_FLAGS    "${ARDUINO_C_FLAGS} -felide-constructors -fno-threadsafe-statics -std=c++11")
#set(ARDUINO_LINKER_FLAGS "")

project(${PROJECT_NAME})

set(${CMAKE_PROJECT_NAME}_SKETCH arduino_st.ino)

#### Uncomment below additional settings as needed.
#set(${CMAKE_PROJECT_NAME}_BOARD uno)
set(${CMAKE_PROJECT_NAME}_PORT /dev/ttyACM0)
# set(mega.build.mcu atmega2560)
# set(mega.upload.protocol wiring)
# set(mega.upload.speed 115200)


#those 2 lines must be added to compile "hello-world" - they add standard sdk
register_hardware_platform(/home/alex/.arduino15/packages/arduino/hardware/avr/1.6.18)
link_directories(/home/alex/.arduino15/packages/arduino/hardware/avr/1.6.18/libraries)


#3rd party libs
link_directories(/home/alex/Work/arduino/libraries)
include_directories(/home/alex/Work/arduino/libraries)

#compatibility with arduino IDE, otherwise it will not find "MPU6050_6Axis_MotionApps20.h"
include_directories(/home/alex/Work/arduino/libraries/MPU6050)

#https://github.com/queezythegreat/arduino-cmake#arduino-libraries
generate_arduino_firmware(${CMAKE_PROJECT_NAME}
        ARDLIBS MPU6050 Wire LiquidCrystal I2CDev Serial)
