# Install script for directory: /Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/llvm-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge/msg" TYPE FILE FILES
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/msg/Marker.msg"
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/msg/Markers.msg"
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/msg/TfDistortInfo.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge/srv" TYPE FILE FILES
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/srv/viconCalibrateSegment.srv"
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/srv/viconGrabPose.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge/cmake" TYPE FILE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/build/vicon_bridge/catkin_generated/installspace/vicon_bridge-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/include/vicon_bridge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/share/roseus/ros/vicon_bridge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/share/common-lisp/ros/vicon_bridge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/share/gennodejs/ros/vicon_bridge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/python3.9" -m compileall "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/python3.9/site-packages/vicon_bridge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.9/site-packages" TYPE DIRECTORY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/python3.9/site-packages/vicon_bridge")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/vicon_bridge" TYPE FILE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/include/vicon_bridge/tf_distortConfig.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.9/site-packages/vicon_bridge" TYPE FILE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/python3.9/site-packages/vicon_bridge/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/python3.9" -m compileall "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/python3.9/site-packages/vicon_bridge/cfg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.9/site-packages/vicon_bridge" TYPE DIRECTORY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/python3.9/site-packages/vicon_bridge/cfg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/build/vicon_bridge/catkin_generated/installspace/vicon_bridge.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge/cmake" TYPE FILE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/build/vicon_bridge/catkin_generated/installspace/vicon_bridge-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge/cmake" TYPE FILE FILES
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/build/vicon_bridge/catkin_generated/installspace/vicon_bridgeConfig.cmake"
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/build/vicon_bridge/catkin_generated/installspace/vicon_bridgeConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge" TYPE FILE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/libvicon_sdk.dylib")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvicon_sdk.dylib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvicon_sdk.dylib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/llvm-strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvicon_sdk.dylib")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge" TYPE EXECUTABLE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/vicon_bridge/vicon_bridge")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/vicon_bridge" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/vicon_bridge")
    execute_process(COMMAND /Users/chiaravongerlach/miniforge3/envs/ros_base/bin/install_name_tool
      -delete_rpath "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/vicon_bridge")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/llvm-strip" -u -r "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/vicon_bridge")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge" TYPE EXECUTABLE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/vicon_bridge/calibrate")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/calibrate" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/calibrate")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/llvm-strip" -u -r "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/calibrate")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge" TYPE EXECUTABLE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/vicon_bridge/tf_distort")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/tf_distort" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/tf_distort")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/llvm-strip" -u -r "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/tf_distort")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge" TYPE EXECUTABLE FILES "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib/vicon_bridge/testclient")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/testclient" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/testclient")
    execute_process(COMMAND /Users/chiaravongerlach/miniforge3/envs/ros_base/bin/install_name_tool
      -delete_rpath "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/devel/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/testclient")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/chiaravongerlach/miniforge3/envs/ros_base/bin/llvm-strip" -u -r "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/vicon_bridge/testclient")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vicon_bridge" TYPE DIRECTORY FILES
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/launch"
    "/Users/chiaravongerlach/Desktop/thesis_slotcar/catkin_ws/src/vicon_bridge/cfg"
    )
endif()

