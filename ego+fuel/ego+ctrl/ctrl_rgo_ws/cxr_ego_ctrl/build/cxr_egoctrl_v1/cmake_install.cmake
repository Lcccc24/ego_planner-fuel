# Install script for directory: /home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/src/cxr_egoctrl_v1

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install" TYPE PROGRAM FILES "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install" TYPE PROGRAM FILES "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/setup.bash;/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install" TYPE FILE FILES
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/setup.bash"
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/setup.sh;/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install" TYPE FILE FILES
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/setup.sh"
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/setup.zsh;/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install" TYPE FILE FILES
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/setup.zsh"
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/install" TYPE FILE FILES "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/cxr_egoctrl_v1.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cxr_egoctrl_v1/cmake" TYPE FILE FILES
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/cxr_egoctrl_v1Config.cmake"
    "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/catkin_generated/installspace/cxr_egoctrl_v1Config-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cxr_egoctrl_v1" TYPE FILE FILES "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/src/cxr_egoctrl_v1/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/uav1/ego+ctrl/ctrl_rgo_ws/cxr_ego_ctrl/build/cxr_egoctrl_v1/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
