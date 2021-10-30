cmake_minimum_required(VERSION 2.8)
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(src/nvilidar)
include_directories(src/impl/serialport/include)
include_directories(src/impl/angles)
include_directories(src/impl/console)
include_directories(src/impl/signal)
include_directories(src/impl/string)
include_directories(src/impl/thread)
include_directories(src/impl/timer)
include_directories(src/impl/socket)


FILE(GLOB SDK_SRC 
  "src/impl/*.h"
  "src/impl/console/*.h"
  "src/impl/signal/*.h"
  "src/impl/string/*.h"
  "src/impl/thread/*.h"
  "src/impl/timer/*.h"
  "src/impl/angles/*.h"
  "src/impl/socket/*.h"
  "src/impl/serialport/include/serial/*.h"
  "src/impl/serialport/include/serial/impl/*.h"
  "src/nvilidar/*.h"
  "src/impl/console/*.cpp"
  "src/impl/timer/*.cpp"
  "src/impl/socket/*.cpp"
  "src/impl/serialport/src/*.cc"
  "src/impl/serialport/src/impl/*.cc"
  "src/impl/serialport/src/impl/list_ports/*.cc"
  "src/nvilidar/*.cpp"
)

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -s")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
add_subdirectory(samples)
add_library(nvilidar_driver SHARED ${SDK_SRC})
IF (WIN32)
target_link_libraries(nvilidar_driver setupapi ws2_32)
ELSE()
target_link_libraries(nvilidar_driver rt pthread)
ENDIF()                    
