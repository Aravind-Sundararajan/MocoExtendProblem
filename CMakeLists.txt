cmake_minimum_required(VERSION 3.2)
project(customGoals VERSION 1.0.1 DESCRIPTION "some custom goals")

set(CMAKE_CXX_STANDARD 11)

find_package(OpenSim 4.3 REQUIRED HINTS
    "${CMAKE_SOURCE_DIR}/../../../../../../")
include("${OpenSim_USE_FILE}")

add_library(osimMocoActivationSquaredGoal SHARED
        ./MocoActivationSquaredGoal/MocoActivationSquaredGoal.h
        ./MocoActivationSquaredGoal/MocoActivationSquaredGoal.cpp
        ./MocoActivationSquaredGoal/osimMocoActivationSquaredGoalDLL.h
        ./MocoActivationSquaredGoal/RegisterTypes_osimMocoActivationSquaredGoal.h
        ./MocoActivationSquaredGoal/RegisterTypes_osimMocoActivationSquaredGoal.cpp
        )
add_library(osimMocoZMPGoal SHARED
        ./MocoZMPGoal/MocoZMPGoal.h
        ./MocoZMPGoal/MocoZMPGoal.cpp
        ./MocoZMPGoal/osimMocoZMPGoalDLL.h
        ./MocoZMPGoal/RegisterTypes_osimMocoZMPGoal.h
        ./MocoZMPGoal/RegisterTypes_osimMocoZMPGoal.cpp
        )
		
add_library(osimMocoCoordinateAccelerationGoal SHARED
        ./MocoCoordinateAccelerationGoal/MocoCoordinateAccelerationGoal.h
        ./MocoCoordinateAccelerationGoal/MocoCoordinateAccelerationGoal.cpp
        ./MocoCoordinateAccelerationGoal/osimMocoCoordinateAccelerationGoalDLL.h
        ./MocoCoordinateAccelerationGoal/RegisterTypes_osimMocoCoordinateAccelerationGoal.h
        ./MocoCoordinateAccelerationGoal/RegisterTypes_osimMocoCoordinateAccelerationGoal.cpp
        )	
		
target_link_libraries(osimMocoActivationSquaredGoal
        osimTools osimExampleComponents osimMoco)
target_link_libraries(osimMocoZMPGoal
        osimTools osimExampleComponents osimMoco)
target_link_libraries(osimMocoCoordinateAccelerationGoal
        osimTools osimExampleComponents osimMoco)
		
set_target_properties(osimMocoActivationSquaredGoal PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(osimMocoActivationSquaredGoal PROPERTIES SOVERSION 1)
set_target_properties(osimMocoActivationSquaredGoal PROPERTIES PUBLIC_HEADER osimMocoActivationSquaredGoal.h)

string(TOUPPER MocoActivationSquaredGoal activation_name_upper)
set_target_properties(osimMocoActivationSquaredGoal PROPERTIES
        DEFINE_SYMBOL OSIM${activation_name_upper}_EXPORTS
        )

set_target_properties(osimMocoZMPGoal PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(osimMocoZMPGoal PROPERTIES SOVERSION 1)
set_target_properties(osimMocoZMPGoal PROPERTIES PUBLIC_HEADER osimMocoZMPGoal.h)

string(TOUPPER MocoZMPGoal zmp_name_upper)
set_target_properties(osimMocoZMPGoal PROPERTIES
        DEFINE_SYMBOL OSIM${zmp_name_upper}_EXPORTS
        )

set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES SOVERSION 1)
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES PUBLIC_HEADER osimMocoCoordinateAccelerationGoal.h)

string(TOUPPER MocoCoordinateAccelerationGoal walk_name_upper)
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES
        DEFINE_SYMBOL OSIM${walk_name_upper}_EXPORTS
        )

install(TARGETS osimMocoActivationSquaredGoal
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
	
install(TARGETS osimMocoZMPGoal
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
	
install(TARGETS osimMocoCoordinateAccelerationGoal
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

foreach(exe testCustomGoal)
    add_executable(${exe} ${exe}.cpp)
    target_link_libraries(${exe}
            osimMocoCoordinateAccelerationGoal)

    # For Windows: make sure DLLs for dependencies are available.
    OpenSimCopyDependencyDLLsForWin(DEP_NAME OpenSim
            DEP_BIN_DIR "${OpenSim_BIN_DIR}")
    if(WIN32)
        add_dependencies(${exe} Copy_OpenSim_DLLs)
    endif()
endforeach()
	
if(WIN32)
    add_dependencies(osimMocoZMPGoal Copy_OpenSim_DLLs)
	add_dependencies(osimMocoCoordinateAccelerationGoal Copy_OpenSim_DLLs)
	add_dependencies(osimMocoActivationSquaredGoal Copy_OpenSim_DLLs)
endif()

file(COPY  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")