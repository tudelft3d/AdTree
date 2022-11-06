cmake_minimum_required(VERSION 3.1)

if (APPLE)
        set(CMAKE_OSX_DEPLOYMENT_TARGET "10.12" CACHE STRING "Minimum OS X deployment version" FORCE)
        message(STATUS "Minimum OS X deployment version: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif ()

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

################################################################################

set (CMAKE_CXX_STANDARD 11)

################################################################################

# Detects whether this is a top-level project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        set(ADTREE_TOPLEVEL_PROJECT ON)
else()
        set(ADTREE_TOPLEVEL_PROJECT OFF)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################

### Configuration
set(ADTREE_ROOT "${CMAKE_CURRENT_LIST_DIR}")
set(ADTREE_EXTERNAL "${ADTREE_ROOT}/3rd_party")

set(ADTREE_glew_INCLUDE_DIR "${ADTREE_EXTERNAL}/glew/include")
set(ADTREE_glfw_INCLUDE_DIR "${ADTREE_EXTERNAL}/glfw/include")
set(ADTREE_imgui_INCLUDE_DIR "${ADTREE_EXTERNAL}/imgui")
set(ADTREE_tetgen_INCLUDE_DIR "${ADTREE_EXTERNAL}/tetgen")
set(ADTREE_kd_tree_INCLUDE_DIR "${ADTREE_EXTERNAL}/kd_tree")
set(ADTREE_cminpack_INCLUDE_DIR "${ADTREE_EXTERNAL}/cminpack")
set(ADTREE_easy3d_INCLUDE_DIR "${ADTREE_EXTERNAL}" "${ADTREE_ROOT}")
set(ADTREE_lm_INCLUDE_DIR "${ADTREE_EXTERNAL}/optimizer_lm")

################################################################################

### conditionally compile certain modules depending on libraries found on the system
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

################################################################################

add_subdirectory(3rd_party)
add_subdirectory(AdTree)

## Resources
# Copy resources dirs into our shadow build directory. For AdTree only the shaders are needed
if (NOT APPLE)
        file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/resources/shaders" DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources")
endif ()

