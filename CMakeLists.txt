# 
#  Copyright (C) 2016 IIT-ADVR
#  Author: Enrico Mingo, Luca Muratore
#  email: enrico.mingo@iit.it, luca.muratore@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

cmake_minimum_required(VERSION 2.8.12)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
    cmake_policy(SET CMP0017 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
PROJECT(XBotCoreModel)

# C++ 11
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

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(srdfdom_advr REQUIRED)
FIND_PACKAGE(urdfdom REQUIRED)
FIND_PACKAGE(kdl_parser REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

INCLUDE_DIRECTORIES(include 
                    ${srdfdom_advr_INCLUDE_DIRS} 
                    ${urdfdom_INCLUDE_DIRS} 
                    ${kdl_parser_INCLUDE_DIRS})

# for every file in INCLUDES CMake already sets the property HEADER_FILE_ONLY
file(GLOB_RECURSE XBotCoreModel_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/include" *.h*)

ADD_LIBRARY(XBotCoreModel SHARED ${XBotCoreModel_INCLUDES} src/XBotCoreModel.cpp)

TARGET_LINK_LIBRARIES(XBotCoreModel ${srdfdom_advr_LIBRARIES} 
                                    ${urdfdom_LIBRARIES}
                                    ${Boost_LIBRARIES}
                                    yaml-cpp
                                    ${kdl_parser_LIBRARIES}
                                    )
                                    
target_include_directories( XBotCoreModel PUBLIC
                                ${srdfdom_advr_INCLUDE_DIRS}
                                ${urdfdom_INCLUDE_DIRS}
                                ${kdl_parser_INCLUDE_DIRS}
                          )


########################################################################

set(VARS_PREFIX "XBotCoreModel")

set(XBotCoreModel_MAJOR_VERSION 0)
set(XBotCoreModel_MINOR_VERSION 2)
set(XBotCoreModel_PATCH_VERSION 0)
set(XBotCoreModel_VERSION ${XBotCoreModel_MAJOR_VERSION}.${XBotCoreModel_MINOR_VERSION}.${XBotCoreModel_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(XBotCoreModel PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                          "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(XBotCoreModel PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                              SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h")

install(TARGETS XBotCoreModel
        EXPORT XBotCoreModel
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)



#enabling it will add all XBotCoreModel dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS XBotCoreModel)


include(InstallBasicPackageFiles)
install_basic_package_files(XBotCoreModel VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)
