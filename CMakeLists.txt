cmake_minimum_required(VERSION 3.2)
project(customGoals VERSION 1.0.2 DESCRIPTION "some custom goals")

set(CMAKE_CXX_STANDARD 11)
set(SRCDIR ${CMAKE_CURRENT_SOURCE_DIR}/custom_goals/)
set(BUILDDIR ${CMAKE_CURRENT_SOURCE_DIR}/build/)
set(LIBDIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/)
set(INCLUDEDIR ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(BINDIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/)
set(TESTDIR ${CMAKE_CURRENT_SOURCE_DIR}/test/)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BINDIR} )

file(MAKE_DIRECTORY ${BUILDDIR})
file(MAKE_DIRECTORY ${BINDIR})
#file(MAKE_DIRECTORY ${LIBDIR})
#file(MAKE_DIRECTORY ${INCLUDEDIR})

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

MACRO(GETTESTS result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*.cpp)
  SET(testlist "")
  FOREACH(child ${children})
      get_filename_component(name ${child} NAME_WE)
      LIST(APPEND testlist ${name})
  ENDFOREACH()
  SET(${result} ${testlist})
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
   #set_target_properties(osim${subdir} PROPERTIES PUBLIC_HEADER osim${subdir}.h)
   set_target_properties(osim${subdir} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${BINDIR})
   set_target_properties(osim${subdir} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${BINDIR})
   set_target_properties(osim${subdir} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BINDIR})
   string(TOUPPER ${subdir} name_upper)
   set_target_properties(osim${subdir} PROPERTIES
        DEFINE_SYMBOL OSIM${name_upper}_EXPORTS
        )
   install(TARGETS osim${subdir}
           LIBRARY DESTINATION ${BINDIR} #CMAKE_INSTALL_LIBDIR
           PUBLIC_HEADER DESTINATION ${BINDIR} #CMAKE_INSTALL_INCLUDEDIR
           ARCHIVE DESTINATION ${BINDIR}
   )
   IF(WIN32)
        add_dependencies(osim${subdir} Copy_OpenSim_DLLs)
   ENDIF()
ENDFOREACH()

GETTESTS(TESTS ${TESTDIR})
FOREACH(exe ${TESTS})
  message(test: ${exe}.exe)
  add_executable(${exe} ${TESTDIR}${exe}.cpp)
  FOREACH(subdir ${SUBDIRS})
      target_link_libraries(${exe} osim${subdir})
      target_include_directories(${exe} PUBLIC ${SRCDIR}${subdir})
      set_target_properties( ${exe}
      PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY ${BINDIR}
      LIBRARY_OUTPUT_DIRECTORY ${BINDIR}
      RUNTIME_OUTPUT_DIRECTORY ${BINDIR}
      )
  ENDFOREACH()
  # For Windows: make sure DLLs for dependencies are available.
  OpenSimCopyDependencyDLLsForWin(DEP_NAME OpenSim
          DEP_BIN_DIR "${OpenSim_BIN_DIR}")
  IF(WIN32)
      add_dependencies(${exe} Copy_OpenSim_DLLs)
  ENDIF()
ENDFOREACH()