get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME 3rd_${MODULE_NAME})
project(${PROJECT_NAME} C)

include(GNUInstallDirs)

if (POLICY CMP0003)
    cmake_policy(SET CMP0003 NEW)
endif ()

if (POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif ()

if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif (POLICY CMP0072)

option(GLEW_X11 "X11 mode" ON)

set(GLEW_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# get version from config/version
file(STRINGS ${GLEW_DIR}/config/version _VERSION_MAJOR_STRING REGEX "GLEW_MAJOR[ ]*=[ ]*[0-9]+.*")
string(REGEX REPLACE "GLEW_MAJOR[ ]*=[ ]*([0-9]+)" "\\1" CPACK_PACKAGE_VERSION_MAJOR ${_VERSION_MAJOR_STRING})
file(STRINGS ${GLEW_DIR}/config/version _VERSION_MINOR_STRING REGEX "GLEW_MINOR[ ]*=[ ]*[0-9]+.*")
string(REGEX REPLACE "GLEW_MINOR[ ]*=[ ]*([0-9]+)" "\\1" CPACK_PACKAGE_VERSION_MINOR ${_VERSION_MINOR_STRING})
file(STRINGS ${GLEW_DIR}/config/version _VERSION_PATCH_STRING REGEX "GLEW_MICRO[ ]*=[ ]*[0-9]+.*")
string(REGEX REPLACE "GLEW_MICRO[ ]*=[ ]*([0-9]+)" "\\1" CPACK_PACKAGE_VERSION_PATCH ${_VERSION_PATCH_STRING})
set(GLEW_VERSION ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH})

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
# prefer GLVND
if (OPENGL_opengl_LIBRARY)
    set(GLEW_LIBRARIES ${OPENGL_opengl_LIBRARY})
else ()
    set(GLEW_LIBRARIES ${OPENGL_gl_LIBRARY})
endif ()
if (OPENGL_glx_LIBRARY)
    list(APPEND GLEW_LIBRARIES ${OPENGL_glx_LIBRARY})
endif ()
message(STATUS "OpenGL libraries: ${GLEW_LIBRARIES}")

# cmake<3.10 doesn't detect EGL/GLX
if (CMAKE_VERSION VERSION_LESS 3.10)
    find_library(OPENGL_egl_LIBRARY NAMES EGL)
    if (OPENGL_egl_LIBRARY)
        set(OpenGL_EGL_FOUND TRUE)
    endif ()
endif ()

# X11 required except for Windows and Apple OSX platforms
if (GLEW_X11 AND NOT WIN32 AND NOT APPLE)
    find_package(X11)
    list(APPEND GLEW_LIBRARIES ${X11_LIBRARIES})
endif ()

add_library(${PROJECT_NAME} STATIC
        ${GLEW_DIR}/include/GL/wglew.h
        ${GLEW_DIR}/include/GL/glew.h
        ${GLEW_DIR}/include/GL/glxew.h
        ${GLEW_DIR}/include/GL/eglew.h
        ${GLEW_DIR}/src/glew.c
        )
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party")

if (MSVC)
    # add options from visual studio project
    target_compile_definitions(${PROJECT_NAME} PRIVATE VC_EXTRALEAN)
    # kill security checks which are dependent on stdlib
    target_compile_options(${PROJECT_NAME} PRIVATE -GS-)
endif ()

target_compile_definitions(${PROJECT_NAME} PUBLIC GLEW_NO_GLU GLEW_STATIC)
target_include_directories(${PROJECT_NAME} PRIVATE ${GLEW_DIR}/include ${X11_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${GLEW_LIBRARIES})
