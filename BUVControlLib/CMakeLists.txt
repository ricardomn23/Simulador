# BUVController Lib

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(ControlBUV)

# ------------- LIBRARY BUILDING  --------------------

# ----- BUVControl ------
add_library(BUV_ControllersLibrary STATIC src/BUVControl.cpp)
target_link_libraries(BUV_ControllersLibrary PUBLIC Eigen3 BUV_SimulatorsLibrary)
target_include_directories(BUV_ControllersLibrary PUBLIC include PRIVATE src)
target_sources(BUV_ControllersLibrary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/BUVControl/BUVControl.hpp)

# C++11 Requirements
target_compile_features(BUV_ControllersLibrary PUBLIC cxx_alias_templates cxx_template_template_parameters cxx_auto_type)

# Warnings and c++ standard compatibility
option(ALL_WARNINGS "Set all compilation warnings on?" OFF)
 if( ALL_WARNINGS )
	if( CMAKE_COMPILER_IS_GNUCXX )
		message("Using g++ compiler.")
		target_compile_options(BUV_ControllersLibrary INTERFACE -ansi -pedantic -Wall)
	elseif( MSVC )
		message("Using Microsoft Visual Studio compiler.")
		target_compile_options(BUV_ControllersLibrary INTERFACE /Za /W4)
	endif(CMAKE_COMPILER_IS_GNUCXX)
endif( ALL_WARNINGS )


# ------------- LIBRARY TESTING  --------------------
option(BUILD_BUVCONTROLLERS_TESTS "Build BUV Simulators tests?" ON)
if ( BUILD_BUVCONTROLLERS_TESTS )
    add_subdirectory ( test )
endif ( BUILD_BUVCONTROLLERS_TESTS )

