#####################################
cmake_minimum_required (VERSION 2.8) 
project (flight_controller)
find_library(WIRINGPI
        NAMES libwiringPi libwiringPi.so
        HINTS "
              ${ROOTFS}/lib ${ROOTFS}/lib/arm-linux-gnueabihf
              ${ROOTFS}/usr/lib ${ROOTFS}/usr/lib/arm-linux-gnueabihf
              "
)
set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH};${CMAKE_INSTALL_PREFIX}/lib/cmake/")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -lncurses -lm")

add_executable (flight_controller flight_controller.cpp)  

target_link_libraries (
        flight_controller
        ${WIRINGPI}
)



#####################################
