# Install script for directory: /home/valen/py3_ws/src/actionlib_tut

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/valen/py3_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tut/action" TYPE FILE FILES "/home/valen/py3_ws/src/actionlib_tut/action/Fibonacci.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tut/msg" TYPE FILE FILES
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciAction.msg"
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciActionGoal.msg"
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciActionResult.msg"
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciActionFeedback.msg"
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciGoal.msg"
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciResult.msg"
    "/home/valen/py3_ws/devel/share/actionlib_tut/msg/FibonacciFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/valen/py3_ws/build/actionlib_tut/catkin_generated/installspace/actionlib_tut.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tut/cmake" TYPE FILE FILES
    "/home/valen/py3_ws/build/actionlib_tut/catkin_generated/installspace/actionlib_tutConfig.cmake"
    "/home/valen/py3_ws/build/actionlib_tut/catkin_generated/installspace/actionlib_tutConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tut" TYPE FILE FILES "/home/valen/py3_ws/src/actionlib_tut/package.xml")
endif()

