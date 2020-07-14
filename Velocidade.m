!bash -c "make"
!bash -c "./bin/BUVSim"
load BUV1_Sim.log
figure
hold on
plot(BUV1_Sim(:,2),sqrt(BUV1_Sim(:,8).^2+BUV1_Sim(:,9).^2+BUV1_Sim(:,10).^2))
xlabel('x(m)');ylabel('v(m/s)'); 
grid

