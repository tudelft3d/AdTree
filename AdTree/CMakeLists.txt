cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


add_executable(${PROJECT_NAME}
                main.cpp
                graph.h
                tree_viewer.h
                tree_viewer.cpp
                skeleton.h
                skeleton.cpp
                cylinder.h
                viewer_imgui.h
                viewer_imgui.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "AdTree")


target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_glew_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_glfw_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_easy3d_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_tetgen_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_kd_tree_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_lm_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)


set (OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
message(STATUS "OpenGL libraries: ${OPENGL_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} glew glfw imgui core util fileio algo viewer tetgen kd_tree cminpack optimizer_lm)


find_package(Boost REQUIRED) # It's "Boost", not "BOOST" or "boost". Case matters.
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS} ${Boost_headers_DIR})
