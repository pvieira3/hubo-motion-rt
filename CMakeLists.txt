cmake_minimum_required(VERSION 2.6)
project(hubo-motion-rt)

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(LIBRARY_INSTALL_PATH /usr/lib/)
set(BINARY_INSTALL_PATH /usr/local/bin/)

set(CMAKE_C_FLAGS "-std=gnu99")

#find_package(ach)
link_libraries(-lm -lrt -lach -lhuboparams -lRobotKin)

file(GLOB daemonizer_source "src/daemonizer.c" "include/daemonizer.h")

file(GLOB drc_hubo_kin_source "src/DrcHuboKin.cpp" "include/DrcHuboKin.h")

file(GLOB control_daemon_source "src/control-daemon.c" "include/control-daemon.h" "tables/control.table")

file(GLOB hubo_control_source "src/Hubo_Control.cpp" "include/Hubo_Control.h")


file(GLOB balance_daemon_source "src/balance-daemon.cpp"
                                "src/Walker.cpp"
                                "include/balance-daemon.h"
                                "include/Walker.h"
                                "include/hubo-zmp.h" )

file(GLOB manip_daemon_source "src/manip-daemon.cpp"
                              "include/manip.h" )

add_library(libdaemonizer SHARED ${daemonizer_source})
link_libraries(libdaemonizer)

add_executable(control-daemon ${control_daemon_source})

add_library(libhuboctrl SHARED ${hubo_control_source})
link_libraries(libhuboctrl)

add_library(libdrchubokin SHARED ${drc_hubo_kin_source})
link_libraries(libdrchubokin)

add_executable(balance-daemon ${balance_daemon_source})

add_executable(manip-daemon ${manip_daemon_source})

# Install Libraries
INSTALL(TARGETS libdaemonizer
                libhuboctrl
        DESTINATION ${LIBRARY_INSTALL_PATH})

# Install Headers
INSTALL(FILES include/Hubo_Control.h 
              include/control-daemon.h 
              include/balance-daemon.h
              include/manip.h
              include/hubo-zmp.h
        DESTINATION /usr/local/include/)

# Install Tables
INSTALL(FILES tables/control.table DESTINATION /etc/hubo-ach/)

# Install Service
INSTALL(PROGRAMS scripts/hubo-motion DESTINATION /etc/init.d/)

# Install Daemons
INSTALL(PROGRAMS control-daemon
                 balance-daemon
                 manip-daemon
        DESTINATION /usr/local/bin/)
