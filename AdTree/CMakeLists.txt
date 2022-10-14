cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


set(${PROJECT_NAME}_SOURCES
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

if (APPLE)
    function(resource VAR SOURCE_PATH DESTINATION PATTERN)
        file(GLOB_RECURSE _LIST CONFIGURE_DEPENDS ${SOURCE_PATH}/${PATTERN})
        foreach (RESOURCE ${_LIST})
            get_filename_component(_PARENT ${RESOURCE} DIRECTORY)
            if (${_PARENT} STREQUAL ${SOURCE_PATH})
                set(_DESTINATION ${DESTINATION})
            else ()
                file(RELATIVE_PATH _DESTINATION ${SOURCE_PATH} ${_PARENT})
                set(_DESTINATION ${DESTINATION}/${_DESTINATION})
            endif ()
            set_property(SOURCE ${RESOURCE} PROPERTY MACOSX_PACKAGE_LOCATION ${_DESTINATION})
        endforeach (RESOURCE)
        set(${VAR} ${_LIST} PARENT_SCOPE)
    endfunction()

    resource(VERTEX_SHADERS ${ADTREE_ROOT}/resources/shaders Resources/shaders *.vert)
    resource(FRAGMENT_SHADERS ${ADTREE_ROOT}/resources/shaders Resources/shaders *.frag)

    # This tells CMake where to find and install the ICON file
    set(${PROJECT_NAME}_ICON ${ADTREE_ROOT}/resources/icons/adtree.icns)

    #Append the new files to the resources file list
    list(APPEND ${PROJECT_NAME}_SOURCES ${${PROJECT_NAME}_ICON} ${VERTEX_SHADERS} ${FRAGMENT_SHADERS})

    add_executable(${PROJECT_NAME} MACOSX_BUNDLE ${${PROJECT_NAME}_ICON} ${${PROJECT_NAME}_SOURCES})

    set_source_files_properties(${${PROJECT_NAME}_ICON} PROPERTIES MACOSX_PACKAGE_LOCATION "Resources")

    ## Note: On macOS, high-DPI support is enabled by settings in the Info.plist file. Make sure they are present.
    ## For more information, see: http://doc.qt.io/qt-5/highdpi.html
    ## Enable high DPI support by using my Info.plist file in which I added NSHighResolutionCapable to be true.
    set_target_properties(${PROJECT_NAME} PROPERTIES
            INSTALL_RPATH "@executable_path/../Frameworks"
            MACOSX_BUNDLE_BUNDLE_NAME ${PROJECT_NAME}
            MACOSX_BUNDLE TRUE
            MACOSX_BUNDLE_INFO_PLIST "${ADTREE_ROOT}/resources/Mac/adtree.plist"
            MACOSX_BUNDLE_ICON_FILE "adtree.icns"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0"
            MACOSX_BUNDLE_LONG_VERSION_STRING "1.0"
            MACOSX_BUNDLE_BUNDLE_VERSION "1.0"
            MACOSX_BUNDLE_GUI_IDENTIFIER "liangliang.nan@gmail.com"
            )
elseif (WIN32 OR MSVC)
	set(${PROJECT_NAME}_RESOURCES ${ADTREE_ROOT}/resources/AdTree.rc)
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_RESOURCES}) # you can add "WIN32" to disable the console window
    message(STATUS "Creating the wxWidgets viewer with a console window")
else ()
    add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
endif ()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "AdTree")

target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_glew_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_glfw_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_easy3d_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_tetgen_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_kd_tree_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${ADTREE_lm_INCLUDE_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE GLEW_STATIC)

#set (OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
message(STATUS "OpenGL libraries: ${OPENGL_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES} 3rd_imgui easy3d_algo easy3d_viewer 3rd_tetgen 3rd_kd_tree 3rd_cminpack 3rd_optimizer_lm)


find_package(Boost REQUIRED) # It's "Boost", not "BOOST" or "boost". Case matters.
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS} ${Boost_headers_DIR})
