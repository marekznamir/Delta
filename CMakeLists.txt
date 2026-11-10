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

if(STM32H7A3xx)
    add_library(Delta STATIC ${SOURCES})
elseif(WIN32)
    add_executable(Delta ${SOURCES})
endif()

if(TARGET Delta)
    target_include_directories(Delta
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/Src
    )
endif()

if(CMAKE_EXPORT_COMPILE_COMMANDS AND CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    add_custom_target(copy_compile_commands ALL
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/compile_commands.json"
            "${CMAKE_CURRENT_SOURCE_DIR}/out/compile_commands.json"
        BYPRODUCTS "${CMAKE_CURRENT_SOURCE_DIR}/out/compile_commands.json"
        COMMENT "Updating out/compile_commands.json"
    )
    if(TARGET Delta)
        add_dependencies(Delta copy_compile_commands)
    endif()
endif()



