cmake_minimum_required(VERSION 3.5)
project(uart_pubsub LANGUAGES CXX)

find_package(Threads REQUIRED)

option(UPS_TEST "Build test app" ON)

set(PLIB_SRC protocol.cpp)
set(PLIB_H protocol.h)
add_library(ulprotocol SHARED ${PLIB_SRC} ${PLIB_H})
target_link_libraries(ulprotocol PUBLIC Threads::Threads)
set_target_properties(ulprotocol PROPERTIES PUBLIC_HEADER ${PLIB_H})
target_compile_options(ulprotocol PUBLIC -fPIC -g -Wall -Wno-unknown-pragmas)

if(UPS_TEST)
    add_executable(ul_test test.cpp)
    target_link_libraries(ul_test PUBLIC ulprotocol)
endif()

if(NOT WIN32)
    include(GNUInstallDirs)
    install(TARGETS ulprotocol
        EXPORT UARTPS
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUEDIR})
    install(EXPORT UARTPS NAMESPACE UARTPS:: FILE UARTPSConfig.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/UARTPS)
endif()
