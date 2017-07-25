# CMake minimum version and policies
cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroYCMInstallLibrary)

project(MiscellaneousPlugin)

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


# C++11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Add cmake dir and include custom macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

set(ENABLE_XENO CACHE BOOL "ON")
if ( ${ENABLE_XENO} )
   find_package(Xenomai)
endif()


# find packages
find_package(XCM REQUIRED)
find_package(OpenSoT QUIET)
find_package(iDynTree QUIET)
find_package(ADVR_ROS QUIET)
find_package(catkin QUIET COMPONENTS roscpp geometry_msgs)

# include
include_directories(include
                    ${XCM_INCLUDE_DIRS}
                    ${OpenSoT_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${ADVR_ROS_INCLUDE_DIRS}
                    )


### Homing
add_library(Homing SHARED src/Homing.cpp)
target_link_libraries(Homing ${XCM_LIBRARIES})
ycm_library_install(Homing 1 0 0)


### BasicFsm
add_library(BasicFsm SHARED src/BasicFsm.cpp)
target_link_libraries(BasicFsm ${XCM_LIBRARIES})
ycm_library_install(BasicFsm 1 0 0)


### OpenSoT IK
if(${OpenSoT_FOUND})
    add_library(OpenSotIk SHARED src/OpenSotIk.cpp)
    target_link_libraries(OpenSotIk ${XCM_LIBRARIES} ${OpenSoT_LIBRARIES} ${iDynTree_LIBRARIES})
    ycm_library_install(OpenSotIk 1 0 0)
endif()

### Grasp
add_library(Grasp SHARED src/Grasp.cpp)
target_link_libraries(Grasp ${XCM_LIBRARIES})
ycm_library_install(Grasp 1 0 0)

### Ros ik IO plugin
if(${catkin_FOUND})

    add_library(IkRosIo SHARED src/IkRosIoPlugin.cpp)
    target_link_libraries(IkRosIo ${XCM_LIBRARIES} ${catkin_LIBRARIES})
    ycm_library_install(IkRosIo 1 0 0)

    add_library(IkRosSMPub SHARED src/IkRosSharedMemoryPublisher.cpp)
    target_link_libraries(IkRosSMPub ${XCM_LIBRARIES} ${catkin_LIBRARIES})
    ycm_library_install(IkRosSMPub 1 0 0)
    
#     if(${ADVR_ROS_FOUND})
    
    add_library(FHRosIo SHARED src/FHRosIoPlugin.cpp)
    target_link_libraries(FHRosIo ${XCM_LIBRARIES} ${catkin_LIBRARIES} ${ADVR_ROS_LIBRARIES})
    ycm_library_install(FHRosIo 1 0 0)

        
#      endif()
    
endif()

### Poses
add_library(Poses SHARED src/Poses.cpp)
target_link_libraries(Poses ${XCM_LIBRARIES})
ycm_library_install(Poses 1 0 0)

### TorqueExample
add_library(TorqueExample SHARED src/TorqueExample.cpp)
target_link_libraries(TorqueExample ${XCM_LIBRARIES})
ycm_library_install(TorqueExample 2 1 0)

if ( Xenomai_FOUND )
    set_xeno_flags(OpenSotIk)
    set_xeno_flags(Grasp)
    set_xeno_flags(IkRosIo)
    set_xeno_flags(IkRosSMPub)
    set_xeno_flags(FHRosIoPlugin)
    set_xeno_flags(FHRosSMPub)
    set_xeno_flags(TorqueExample)
endif()

include(AddUninstallTarget)
