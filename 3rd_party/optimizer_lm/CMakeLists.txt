cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})



add_library(${PROJECT_NAME} STATIC 
	optimizer_lm.h
	optimizer_lm.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      FOLDER "3rd_party")


target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_cminpack_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} cminpack)
