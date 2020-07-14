!bash -c "make"
!bash -c "./bin/BUVSim"
load BUV1_Sim.log
load target.log
figure
hold on
plot(BUV1_Sim(2:end-1,1),sqrt((target(2:end,1)-target(1,1)).^2+(target(2:end,2)-target(1,2)).^2+(target(2:end,3)-target(1,3)).^2)./BUV1_Sim(2:end-1,1));
plot(BUV1_Sim(:,1),sqrt(BUV1_Sim(:,8).^2+BUV1_Sim(:,9).^2+BUV1_Sim(:,10).^2));
xlabel('x(m)');ylabel('v(m/s)');