!bash -c "make"
!bash -c "./bin/BUVSim"
load BUV1_Sim.log
load target.log
figure
plot(BUV1_Sim(2:end,1),sqrt((target(:,1)-BUV1_Sim(2:end,2)).^2+(target(:,2)-BUV1_Sim(2:end,3)).^2+(target(:,3)-BUV1_Sim(2:end,4)).^2));
xlabel('t(s)');ylabel('distancia(m)');
grid