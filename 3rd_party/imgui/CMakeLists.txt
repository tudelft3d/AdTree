cmake_minimum_required(VERSION 3.1)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME 3rd_${MODULE_NAME})
project(${PROJECT_NAME})


set(imgui_HEADERS
    imgui.h
    impl/imgui_impl_glfw.h
    impl/imgui_impl_opengl3.h
    )

set(imgui_SOURCES
    imgui.cpp
    imgui_demo.cpp
    imgui_draw.cpp
    imgui_widgets.cpp
    impl/imgui_impl_glfw.cpp
    impl/imgui_impl_opengl3.cpp
    )


add_library(${PROJECT_NAME} STATIC ${imgui_SOURCES} ${imgui_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party")

set(IMGUI_INCLUDE_DIR  ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${ADTREE_imgui_INCLUDE_DIR} ${ADTREE_glew_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)
target_link_libraries(${PROJECT_NAME} 3rd_glew 3rd_glfw)


if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _CRT_SECURE_NO_DEPRECATE
        )
endif()

