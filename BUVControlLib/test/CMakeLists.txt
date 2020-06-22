# BUV Simulators Demos

# -------------   DEMO BUILDING   --------------------
#
# BUV Simulators Demonstration Executables
#

add_executable(BUV_ControlTest BUVControlTest.cpp)
target_link_libraries(BUV_ControlTest PRIVATE Eigen3 BUV_SimulatorsLibrary BUV_ControllersLibrary)
