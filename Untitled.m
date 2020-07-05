!bash -c "make"
!bash -c "./bin/BUVSim"
load BUV1_Sim.log
load target.log
figure
hold on
plot3(target(:,1),target(:,2),target(:,3));
plot3(BUV1_Sim(:,2),BUV1_Sim(:,3),BUV1_Sim(:,4)); grid
hold off
xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');