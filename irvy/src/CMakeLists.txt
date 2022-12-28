cmake_minimum_required(VERSION 3.12)
project(irvy)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# find_path(pigpio_INCLUDE_DIR 
# 	NAMES pigpio.h pigpiod_if.h pigpiod_if2.h
# 	HINTS /usr/local/include)

# find_library(pigpio_LIBRARY 
# 	NAMES libpigpio.so
# 	HINTS /usr/local/lib)
# find_library(pigpiod_if_LIBRARY 
# 	NAMES libpigpiod_if.so
# 	HINTS /usr/local/lib)
# find_library(pigpiod_if2_LIBRARY 
# 	NAMES libpigpiod_if2.so
# 	HINTS /usr/local/lib)

# set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
# set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

# include(FindPackageHandleStandardArgs)
# find_package_handle_standard_args(pigpio 
#     DEFAULT_MSG 
#     pigpio_INCLUDE_DIR pigpio_LIBRARY pigpiod_if_LIBRARY pigpiod_if2_LIBRARY)

add_executable(
    irvy
    main.cpp
)

target_link_libraries(
    irvy
    Threads::Threads
    # pigpio
    ${OpenCV_LIBS}
)