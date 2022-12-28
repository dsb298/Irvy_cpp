cmake_minimum_required(VERSION 3.10)
project(base)

# Find required libraries
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Add include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(
    base
    main.cpp
)

# Link libraries
target_link_libraries(
    base
    ncurses
    Threads::Threads
    ${OpenCV_LIBS}
    ${MicroHTTPD_LIBS}
)