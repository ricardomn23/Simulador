# BUV Simulators Demos

# -------------   DEMO BUILDING   --------------------
#
# BUV Simulators Demonstration Executables
#

add_executable(BUV_SimulatorTest BUVSimTest.cpp)
target_link_libraries(BUV_SimulatorTest PRIVATE BUV_SimulatorsLibrary)
