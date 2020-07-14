!bash -c "make"
!bash -c "./bin/BUVSim"
load BUV1_Sim.log
figure
hold on
plot3(BUV1_Sim(:,2),BUV1_Sim(:,3),BUV1_Sim(:,4)); grid
xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
