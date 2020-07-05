!bash -c "make"
!bash -c "./bin/BUVSim"
load BUV1_Sim.log
plot3(BUV1_Sim(:,2),BUV1_Sim(:,3),BUV1_Sim(:,4));
