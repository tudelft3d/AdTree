cmake_minimum_required(VERSION 3.1)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME 3rd_${MODULE_NAME})
project(${PROJECT_NAME})


set(${PROJECT_NAME}_HEADERS
    graph_io.h
    ply_reader_writer.h
    point_cloud_io.h
    surface_mesh_io.h

    )

set(${PROJECT_NAME}_SOURCES
    graph_io.cpp
    graph_io_ply.cpp
    ply_reader_writer.cpp
    point_cloud_io.cpp
    point_cloud_io_xyz.cpp
    surface_mesh_io.cpp
    surface_mesh_io_obj.cpp
    )

	
add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS} )

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party/easy3d")

target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_easy3d_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} 3rd_core 3rd_rply 3rd_util)

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _CRT_SECURE_NO_DEPRECATE)
endif()
