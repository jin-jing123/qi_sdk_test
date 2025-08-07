cmake_minimum_required(VERSION 3.10)
project(qi_sdk LANGUAGES C CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_EXAMPLES "Build examples" ON)



#set(default_build_type "Release")
#if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
#    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
#    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
#            STRING "Choose the type of build." FORCE)
#    # Set the possible values of build type for cmake-gui
#    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
#            "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
#endif ()

## Use GNUInstallDirs to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

## Put all binary files into /bin and libraries into /lib
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

## Check system architecture
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

## Import thirdparty libraries
add_subdirectory(thirdparty)

set(QI_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})
find_library(QI_SDK_LIB qi_sdk PATHS ${QI_SDK_PATH} NO_DEFAULT_PATH)

if (NOT QI_SDK_LIB)
    message(FATAL_ERROR "Qi SDK library for the architecture is not found")
else ()
    message(STATUS "QI SDK library found at: ${QI_SDK_LIB}")
endif ()

message(STATUS "Importing: ${QI_SDK_LIB}")

find_package(Threads REQUIRED)

add_library(qi_sdk STATIC IMPORTED GLOBAL)
set_target_properties(qi_sdk PROPERTIES
        IMPORTED_LOCATION ${QI_SDK_LIB})
target_link_libraries(qi_sdk INTERFACE ddsc ddscxx Threads::Threads)
target_include_directories(qi_sdk INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qi/idl>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/qi/robot>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

if (BUILD_EXAMPLES)
    add_subdirectory(example)
endif ()

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${QI_SDK_LIB}
    DESTINATION ${CMAKE_INSTALL_LIBDIR})

#install(TARGETS qi_sdk
#    EXPORT qi_sdkTargets
#    #ARCHIVE DESTINATION lib/${CMAKE_SYSTEM_PROCESSOR}
#    #LIBRARY DESTINATION lib/${CMAKE_SYSTEM_PROCESSOR}
#    #RUNTIME DESTINATION bin
#    #INCLUDES DESTINATION include
#)
# 安装导出的 Targets 文件
#install(
#  EXPORT qi_sdkTargets
#  FILE    qi_sdkTargets.cmake
#  #NAMESPACE qi_sdk::              # 推荐给目标加个 namespace
#  DESTINATION lib/cmake/qi_sdk
#)

#install(
#    FILES lib/cmake/qi_sdk/qi_sdkTargets.cmake
#    DESTINATION lib/cmake/qi_sdk
#)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    qi_sdkConfigVersion.cmake
    VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}"
    COMPATIBILITY ExactVersion)

configure_file(lib/cmake/qi_sdk/qi_sdkConfig.cmake.in qi_sdkConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/qi_sdkConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/qi_sdkConfigVersion.cmake"
    DESTINATION lib/cmake/qi_sdk)

# configure_file(lib/cmake/qi_sdk/qi_sdkTargets.cmake.in qi_sdkTargets.cmake @ONLY)
install(FILES "lib/cmake/qi_sdk/qi_sdkTargets.cmake"
    DESTINATION lib/cmake/qi_sdk)
