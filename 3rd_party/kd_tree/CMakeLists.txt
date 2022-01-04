cmake_minimum_required(VERSION 3.1)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME 3rd_${MODULE_NAME})
project(${PROJECT_NAME})



add_library(${PROJECT_NAME} STATIC 
        KdTree.h
        KdTree.cpp
	PriorityQueue.h
	PriorityQueue.cpp
	Vector2D.h
	Vector2D.cpp
	Vector3D.h
	Vector3D.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      FOLDER "3rd_party")


target_include_directories(${PROJECT_NAME} PRIVATE ${EASY3D_kd_INCLUDE_DIR})
