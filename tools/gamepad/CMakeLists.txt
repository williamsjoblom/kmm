
cmake_minimum_required(VERSION 2.6)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# Enable debug symbols by default
# must be done before project() statement
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()
# (you can also set it on the command line: -D CMAKE_BUILD_TYPE=Release)

project(gamepad)

include_directories("${PROJECT_BINARY_DIR}")

# Define sources and executable
set(EXECUTABLE_NAME "gamepad")
add_executable(${EXECUTABLE_NAME}
  main.cpp
)

set(
  CMAKE_MODULE_PATH
  cmake_modules
  "${CMAKE_SOURCE_DIR}/cmake_modules"
  ${CMAKE_MODULE_PATH}
  /usr/local/share/SFML/cmake/Modules/
  /usr/local/share/cmake/Modules/
)

find_package(SFML 2 REQUIRED system window graphics network audio)

include_directories(${SFML_INCLUDE_DIR})

target_link_libraries(
  ${EXECUTABLE_NAME}
  ${SFML_LIBRARIES}
)

if(NOT APPLE)
    target_link_libraries(
        ${EXECUTABLE_NAME}
        -lX11
        )
endif(NOT APPLE)

# Install target
install(TARGETS ${EXECUTABLE_NAME} DESTINATION bin)

# CPack packaging
include(InstallRequiredSystemLibraries)
include(CPack)
