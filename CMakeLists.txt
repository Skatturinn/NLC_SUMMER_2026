cmake_minimum_required(VERSION 3.8)
project(imu_sync_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the required Qt5 dependencies
find_package(Qt5 COMPONENTS Core SerialPort REQUIRED)

# Point exactly to where your SDK lives, based on your working project
# set(MYCOBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../MyCobotCpplib")
set(MYCOBOT_SDK_ROOT "/home/jetson/jetcobot_humble_ws/src/MyCobotCppLib")
set(MYCOBOT_INCLUDE_DIR "${MYCOBOT_SDK_ROOT}/include")
set(MYCOBOT_LIB "${MYCOBOT_SDK_ROOT}/build/lib/libmyCobotCpp.so")

# Verify the SDK exists before trying to compile
if(NOT EXISTS "${MYCOBOT_INCLUDE_DIR}/mycobot/MyCobot.hpp")
  message(FATAL_ERROR "Cannot find mycobot/MyCobot.hpp under ${MYCOBOT_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${MYCOBOT_LIB}")
  message(FATAL_ERROR "Cannot find libmyCobotCpp.so at ${MYCOBOT_LIB}")
endif()

message(STATUS "Using SDK include: ${MYCOBOT_INCLUDE_DIR}")
message(STATUS "Using SDK lib: ${MYCOBOT_LIB}")

# Create the executable from your script
add_executable(s002 s002.cpp)

# Include the myCobot headers
target_include_directories(s002 PRIVATE 
    ${MYCOBOT_INCLUDE_DIR} 
)

# Link the libraries
target_link_libraries(s002 
    ${MYCOBOT_LIB} 
    Qt5::Core 
    Qt5::SerialPort
)

# Set the RPATH so the executable knows exactly where to find libmyCobotCpp.so at runtime
set_target_properties(s002 PROPERTIES
  BUILD_RPATH "${MYCOBOT_SDK_ROOT}/build/lib"
  INSTALL_RPATH "${MYCOBOT_SDK_ROOT}/build/lib"
)
