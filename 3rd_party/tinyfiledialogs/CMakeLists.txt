cmake_minimum_required(VERSION 3.12)

get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(PROJECT_NAME 3rd_${MODULE_NAME})
project(${PROJECT_NAME})


set(tinyfiledialogs_HEADERS
    tinyfiledialogs.h
    )

set(tinyfiledialogs_SOURCES
    tinyfiledialogs.c
    )


add_library(${PROJECT_NAME} STATIC ${tinyfiledialogs_SOURCES} ${tinyfiledialogs_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd_party")

if (MSVC)
  target_compile_definitions(${PROJECT_NAME} PRIVATE
      _CRT_SECURE_NO_WARNINGS
      _CRT_SECURE_NO_DEPRECATE
      )
endif()
