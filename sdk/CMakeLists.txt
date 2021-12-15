cmake_minimum_required(VERSION 2.8)
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(src/nvilidar)
include_directories(src/impl/include)


FILE(GLOB SDK_SRC 
  "src/impl/include/*.h"
  "src/impl/include/serial/*.h"
  "src/impl/include/socket/*.h"
  "src/nvilidar/*.h"
  "src/impl/src/serial/*.cpp"
  "src/impl/src/socket/*.cpp"
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
