cmake_minimum_required(VERSION 3.1)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME easy3d_${MODULE_NAME})
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
        remove_duplication.h
        )

set(${PROJECT_NAME}_SOURCES
        remove_duplication.cpp
        )


add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party/easy3d")

target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_easy3d_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} easy3d_core easy3d_util)

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE)
endif ()
