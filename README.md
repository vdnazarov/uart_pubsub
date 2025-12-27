# About
This is a small PUB-SUB over UART implementation for Linux systems

# Compatabillity
Library should work under any Linux based system. It tested with
 - Rasberry Pi 5 with trixie based OS
 - Orange Pi 5 plus with bookworm based OS

# Requirements
 - `CMake` 3.5 or later
 - build-essentials (`gcc`, `make`, cmake compatable threading library, such as `pthread`)

# Standalone build
```bash
mkdir build
cd build
cmake .. && make && sudo make install
```

`CMake` options:
 - `UPS_TEST` - boolean, if `ON` - test application will be build

In your `CMakeList.txt` files library can be link as
```cmake
find_package(UARTPS REQUIRED)

add_executable(example_app main.cpp)
target_link_libraries(exapmple_app PUBLIC UARTPS::ulprotocol)
```

# FetchContetnt
```cmake
FetchContetnt_Declare(UARTPS SYSTEM
  GIT_REPOSITORY https://github.com/vdnazarov/uart_pubsub
  GIT_TAG v1.0
  GIT_SHALLOW TRUE)
set(UPS_TEST OFF CACHE BOOL "" FORCE)
FetchContetnt_MakeAvailable(UARTPS)

add_executable(example_app main.cpp)
target_link_libraries(exapmple_app PUBLIC ulprotocol)
```
