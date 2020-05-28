# BUVSim Lib

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(SimulatorBUV)

# ------------- LIBRARY BUILDING  --------------------

# ----- Eigen -------
add_library(Eigen3 INTERFACE)
target_include_directories(Eigen3 INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/Eigen3)


# ----- BUVSim ------
add_library(BUV_SimulatorsLibrary STATIC src/BUVSim.cpp)
target_link_libraries(BUV_SimulatorsLibrary PUBLIC Eigen3)
target_include_directories(BUV_SimulatorsLibrary PUBLIC include PRIVATE src)
target_sources(BUV_SimulatorsLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/BUVSim/BUVSim.hpp)


# C++11 Requirements
target_compile_features(BUV_SimulatorsLibrary PUBLIC cxx_alias_templates cxx_template_template_parameters cxx_auto_type)

# Warnings and c++ standard compatibility
option(ALL_WARNINGS "Set all compilation warnings on?" OFF)
 if( ALL_WARNINGS )
	if( CMAKE_COMPILER_IS_GNUCXX )
		message("Using g++ compiler.")
		target_compile_options(BUV_SimulatorsLibrary INTERFACE -ansi -pedantic -Wall)
	elseif( MSVC )
		message("Using Microsoft Visual Studio compiler.")
		target_compile_options(BUV_SimulatorsLibrary INTERFACE /Za /W4)
	endif(CMAKE_COMPILER_IS_GNUCXX)
endif( ALL_WARNINGS )


# ------------- LIBRARY TESTING  --------------------
option(BUILD_BUVSIMULATORS_TESTS "Build BUV Simulators tests?" ON)
if ( BUILD_BUVSIMULATORS_TESTS )
    add_subdirectory ( test )
endif ( BUILD_BUVSIMULATORS_TESTS )

