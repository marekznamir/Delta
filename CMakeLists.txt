cmake_minimum_required(VERSION 3.16)
project(Delta VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE DISPLAY_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c
)

set(SOURCES
    ${DISPLAY_SOURCES}
)

if(STM32H7A3xx OR TARGET stm32cubemx OR CMAKE_CROSSCOMPILING)
    add_library(Delta STATIC ${SOURCES})

    target_include_directories(Delta PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)

    target_link_libraries(Delta PRIVATE stm32cubemx)
elseif(WIN32)
    add_executable(Delta ${SOURCES})

    target_include_directories(Delta PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Inc)
endif()
