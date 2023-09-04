%% 畫圖
figure(1)
title('foot Trajectory X');hold on;
plot(Time, Lx,'m-');hold on;
plot(Time, Rx,'g-');hold on;
xlabel('time(ms)');
ylabel('x(cm)');
legend('Lx','Rx')

figure(2)
title('foot Trajectory Y');hold on;
plot(Time, Ly,'m-');hold on;
plot(Time, Ry,'g-');hold on;
xlabel('time(ms)');
ylabel('y(cm)');
legend('Ly','Ry')

figure(3)
title('foot Trajectory Z');hold on;
plot(Time, Lz,'m-');hold on;
plot(Time, Rz,'g-');hold on;
xlabel('time(ms)');
ylabel('z(cm)');
legend('Lz','Rz')

figure(4)
title('Com Trajectory X');hold on;
plot(Time, Cpx);
xlabel('time(ms)');
ylabel('x(cm)');

figure(5)
title('Com Trajectory Y');hold on;
plot(Time, Cpy);
xlabel('time(ms)');
ylabel('y(cm)');

figure(6)
title('Com Trajectory');hold on;
plot(Cpx, Cpy);
xlabel('x(cm)');
ylabel('y(cm)');

figure(7)
title('Simulate Walkinggait')
plot3(Lx,Ly,Lz,'m-');hold on;
plot3(Rx,Ry,Rz,'g-');hold on;
plot3(Cpx,Cpy,Cpz,'ro-');hold on;
plot3(zmpx,zmpy,zmpz,'b--');hold on;
plot3(zmpx,zmpy,zmpz,'K*','LineWidth',3);hold on;
xx=[zmpx;Cpx];
yy=[zmpy;Cpy];
zz=[zmpz;Cpz];
plot3(xx,yy,zz,'K-');
grid on;axis normal;
xlabel('x(cm)')
ylabel('y(cm)')
zlabel('z(cm)')
legend('leftfoot','rightfoot','Com','ZMP')

figure(8)
subplot(2,1,1)
plot(Time,zmpx,'B--','LineWidth',1);hold on;
plot(Time,Cpx,'R','LineWidth',1);title('Position X');grid on;
subplot(2,1,2)
plot(Time,Cvx,'R','LineWidth',1');title('Velocity X');hold on;
grid on;

figure(9)
subplot(2,1,1)
plot(Time,Cpy,'R','LineWidth',1);title('Position Y');hold on;
plot(Time,zmpy,'B--','LineWidth',1);title('Position Y');hold on;
grid on;
subplot(2,1,2)
plot(Time,Cvy,'R','LineWidth',1');title('Velocity Y');hold on;
grid on;

figure(10)
title('x axis-robot coordinates');hold on;
plot(Time,end_point_lx);hold on;
plot(Time,end_point_rx);hold on;
grid on;axis normal;
xlabel('x(cm)')
ylabel('y(cm)')
legend('end point l','end point r')

figure(11)
title('x axis-robot coordinates');hold on;
plot(Time,end_point_ly);hold on;
plot(Time,end_point_ry);hold on;
grid on;axis normal;
xlabel('x(cm)')
ylabel('y(cm)')
legend('end point l','end point r')

figure(12)
title('x axis-robot coordinates');hold on;
plot(Time,end_point_lz);hold on;
plot(Time,end_point_rz);hold on;
grid on;axis normal;
xlabel('x(cm)')
ylabel('y(cm)')
legend('end point l','end point r')