cmake_minimum_required(VERSION 3.10)
project(client)

# Find required libraries
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Add include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(
    client
    client.cpp
)

# Link libraries
target_link_libraries(
    client
    ncurses
    Threads::Threads
    ${OpenCV_LIBS}
    ${MicroHTTPD_LIBS}
)