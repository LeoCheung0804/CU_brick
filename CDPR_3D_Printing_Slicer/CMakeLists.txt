cmake_minimum_required(VERSION 3.16)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_CURRENT_BINARY_DIR)
    message(FATAL_ERROR "Use a separate build directory inside this project's build/ directory.")
endif()

file(RELATIVE_PATH _cdpr_build_path
    "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}")
if(NOT _cdpr_build_path MATCHES "^build(/|$)")
    message(FATAL_ERROR "Keep generated files in CDPR_3D_Printing_Slicer/build/.")
endif()

project(CDPR_3D_Printing_Slicer VERSION 0.1.0 LANGUAGES CXX)

option(CDPR_BUILD_HARDWARE_HOST "Build the separately approved controller host" OFF)
option(BUILD_TESTING "Build the separately approved offline tests" OFF)

if(CDPR_BUILD_HARDWARE_HOST)
    message(FATAL_ERROR "The hardware host is not implemented. Controller integration requires separate approval.")
endif()

get_filename_component(CDPR_REPOSITORY_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

foreach(_cdpr_required_file IN ITEMS
    "include/CableKinematics.h"
    "src/CableKinematics.cpp"
    "Dependencies/eigen-3.3.7/Eigen/Core"
    "tools/json.hpp")
    if(NOT EXISTS "${CDPR_REPOSITORY_ROOT}/${_cdpr_required_file}")
        message(FATAL_ERROR "Missing existing repository dependency: ${_cdpr_required_file}")
    endif()
endforeach()

add_library(cdpr_slicer_contracts INTERFACE)
add_library(cdpr_slicer::contracts ALIAS cdpr_slicer_contracts)
target_compile_features(cdpr_slicer_contracts INTERFACE cxx_std_14)
target_include_directories(cdpr_slicer_contracts INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CDPR_REPOSITORY_ROOT}/include")
target_include_directories(cdpr_slicer_contracts SYSTEM INTERFACE
    "${CDPR_REPOSITORY_ROOT}/Dependencies/eigen-3.3.7")

# These interfaces expose existing headers without modifying or copying them.
add_library(cdpr_slicer_json INTERFACE)
target_include_directories(cdpr_slicer_json INTERFACE
    "${CDPR_REPOSITORY_ROOT}/tools")
add_library(cdpr_slicer::json ALIAS cdpr_slicer_json)

# Shared mathematical source, without Robot, COM, ADS or Teknic. It is excluded
# as a standalone target but built when the planning target depends on it.
add_library(cdpr_slicer_kinematics STATIC EXCLUDE_FROM_ALL
    "${CDPR_REPOSITORY_ROOT}/src/CableKinematics.cpp")
add_library(cdpr_slicer::kinematics ALIAS cdpr_slicer_kinematics)
target_compile_features(cdpr_slicer_kinematics PUBLIC cxx_std_14)
target_include_directories(cdpr_slicer_kinematics PUBLIC
    "${CDPR_REPOSITORY_ROOT}/include")
set_target_properties(cdpr_slicer_kinematics PROPERTIES
    CXX_EXTENSIONS OFF
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

add_library(cdpr_slicer_import STATIC
    src/gcode/GCodeParser.cpp
    src/geometry/CoordinateTransforms.cpp
    src/io/PrintJobIO.cpp)
add_library(cdpr_slicer::import ALIAS cdpr_slicer_import)
target_link_libraries(cdpr_slicer_import PUBLIC cdpr_slicer::contracts
    PRIVATE cdpr_slicer::json)
set_target_properties(cdpr_slicer_import PROPERTIES
    CXX_EXTENSIONS OFF
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

add_library(cdpr_slicer_planning STATIC
    src/cdpr/RobotModelLoader.cpp
    src/cdpr/KinematicsAdapter.cpp
    src/planning/WorkspaceValidator.cpp
    src/planning/TrajectoryPlanner.cpp
    src/planning/MechanicsAssessment.cpp
    src/visualization/PreviewWriter.cpp)
add_library(cdpr_slicer::planning ALIAS cdpr_slicer_planning)

# Bundle the local renderer into the executable; exported previews stay standalone.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/PreviewTemplate.html"
    CDPR_SLICER_PREVIEW_TEMPLATE)
# MSVC limits individual string literals; retain newlines but split each line
# into an adjacent raw literal instead of one large literal.
string(REPLACE "\n" "\n)CDPR_HTML\"\nR\"CDPR_HTML("
    CDPR_SLICER_PREVIEW_TEMPLATE "${CDPR_SLICER_PREVIEW_TEMPLATE}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/PreviewTemplate.html")
configure_file(src/visualization/PreviewTemplate.h.in
    "${CMAKE_CURRENT_BINARY_DIR}/generated/PreviewTemplate.h" @ONLY)
target_include_directories(cdpr_slicer_planning PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/generated")
target_link_libraries(cdpr_slicer_planning PUBLIC
    cdpr_slicer::import cdpr_slicer::kinematics PRIVATE cdpr_slicer::json)
set_target_properties(cdpr_slicer_planning PROPERTIES CXX_EXTENSIONS OFF
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")

# The bounded file importer uses Windows file handles, not controller APIs.
if(WIN32)
    add_executable(cdpr-slicer-plan src/cli/PlanMain.cpp)
    target_link_libraries(cdpr-slicer-plan PRIVATE
        cdpr_slicer::planning cdpr_slicer::json)
    target_compile_definitions(cdpr-slicer-plan PRIVATE
        CDPR_SLICER_PROJECT_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
    set_target_properties(cdpr-slicer-plan PROPERTIES CXX_EXTENSIONS OFF
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
    if(MINGW)
        target_link_options(cdpr-slicer-plan PRIVATE -municode)
    endif()
else()
    message(STATUS "File CLI/model loading target Windows; numerical APIs and nine test groups use portable C++14.")
endif()

if(BUILD_TESTING)
    enable_testing()
    add_executable(cdpr-slicer-offline-tests tests/OfflineTests.cpp)
    target_link_libraries(cdpr-slicer-offline-tests PRIVATE
        cdpr_slicer::planning cdpr_slicer::json)
    target_compile_definitions(cdpr-slicer-offline-tests PRIVATE
        CDPR_SLICER_FIXTURE_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
    target_compile_definitions(cdpr-slicer-offline-tests PRIVATE
        CDPR_SLICER_TEST_DIRECTORY="${CMAKE_CURRENT_BINARY_DIR}/test-data")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/test-data")
    set_target_properties(cdpr-slicer-offline-tests PROPERTIES CXX_EXTENSIONS OFF
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
    add_test(NAME cdpr-slicer-offline COMMAND cdpr-slicer-offline-tests)
    if(WIN32)
        add_executable(cdpr-slicer-mechanics-tests tests/MechanicsTests.cpp)
        target_link_libraries(cdpr-slicer-mechanics-tests PRIVATE cdpr_slicer::planning cdpr_slicer::json)
        target_compile_definitions(cdpr-slicer-mechanics-tests PRIVATE CDPR_SLICER_PROJECT_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
        set_target_properties(cdpr-slicer-mechanics-tests PROPERTIES CXX_EXTENSIONS OFF RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
        add_test(NAME cdpr-slicer-mechanics COMMAND cdpr-slicer-mechanics-tests)
        if(MSVC)
            target_compile_options(cdpr-slicer-mechanics-tests PRIVATE /utf-8 /W4)
        endif()
    endif()
endif()

if(MSVC)
    target_compile_options(cdpr_slicer_import PRIVATE /utf-8 /W4)
    target_compile_options(cdpr_slicer_planning PRIVATE /utf-8 /W4)
    if(TARGET cdpr-slicer-plan)
        target_compile_options(cdpr-slicer-plan PRIVATE /utf-8 /W4)
    endif()
    if(TARGET cdpr-slicer-offline-tests)
        target_compile_options(cdpr-slicer-offline-tests PRIVATE /utf-8 /W4)
    endif()
endif()

# Declaring these targets does not authorize configuring, building or running.
# No dependency downloads, install rules or hardware targets are provided.
