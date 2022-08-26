cmake_minimum_required(VERSION 3.2)
project(customGoals VERSION 1.0.1 DESCRIPTION "some custom goals")

set(CMAKE_CXX_STANDARD 11)
set(SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/custom_goals/)
set(BUILDDIR ${CMAKE_CURRENT_SOURCE_DIR}/build/)
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/)
set(BINDIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/)
file(MAKE_DIRECTORY ${BUILDDIR})
file(MAKE_DIRECTORY ${BINDIR})
file(MAKE_DIRECTORY ${LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BINDIR})
set(CMAKE_CURRENT_BINARY_DIR ${BUILDDIR})

find_package(OpenSim 4.3)
include("${OpenSim_USE_FILE}")

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

message("looking for custom goals:")
SUBDIRLIST(SUBDIRS ${SRCDIR})
FOREACH(subdir ${SUBDIRS})
  message(osim${subdir}:)
  add_library(osim${subdir} SHARED
        ${SRCDIR}/${subdir}/${subdir}.h
        ${SRCDIR}/${subdir}/${subdir}.cpp
        ${SRCDIR}/${subdir}/osim${subdir}DLL.h
        ${SRCDIR}/${subdir}/RegisterTypes_osim${subdir}.h
        ${SRCDIR}/${subdir}/RegisterTypes_osim${subdir}.cpp
        )
   target_link_libraries(osim${subdir} osimTools osimExampleComponents osimMoco)
   set_target_properties(osim${subdir} PROPERTIES VERSION ${PROJECT_VERSION})
   set_target_properties(osim${subdir} PROPERTIES SOVERSION 1)
   set_target_properties(osim${subdir} PROPERTIES PUBLIC_HEADER osim${subdir}.h)
   string(TOUPPER ${subdir} name_upper)
   set_target_properties(osim${subdir} PROPERTIES
        DEFINE_SYMBOL OSIM${name_upper}_EXPORTS
        )
   install(TARGETS osim${subdir}
           LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
           PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
   IF(WIN32)
        add_dependencies(osim${subdir} Copy_OpenSim_DLLs)
   ENDIF()
ENDFOREACH()

FOREACH(exe testCustomGoal)
    add_executable(${exe} ${exe}.cpp)
    FOREACH(subdir ${SUBDIRS})
        target_link_libraries(${exe} osim${subdir})
    ENDFOREACH()
    # For Windows: make sure DLLs for dependencies are available.
    OpenSimCopyDependencyDLLsForWin(DEP_NAME OpenSim
            DEP_BIN_DIR "${OpenSim_BIN_DIR}")
    IF(WIN32)
        add_dependencies(${exe} Copy_OpenSim_DLLs)
    ENDIF()
ENDFOREACH()