%% 四足walk步态移动的角度曲线，配合adams使用
clear;
clc;

%% 不同运动顺序测试
% 读取参数测试顺序准备->前进->后退->左移->右移
A=readmatrix('prepare.txt');  %准备
B=readmatrix('forward.txt');  %前进
C=readmatrix('back.txt');     %后退
D=readmatrix('left.txt');     %左移
E=readmatrix('right.txt');    %右移
%F=readmatrix('tabu.txt');    %踏步
angle=[A;B;C;D;E];

writematrix(angle(:,1:13),'T_angle.txt','Delimiter','tab');
t=angle(:,13);
 %t=0.001:0.001:size(angle(:,1))/1000; %1ms执行一次

%% 输出关节角度曲线
leg1_theta1 = angle(:,1);
leg1_theta2 = angle(:,2);
leg1_theta3 = angle(:,3);
subplot(331);
plot(t,leg1_theta1,'r',t,leg1_theta2,'b',t,leg1_theta3,'g');

leg2_theta1 = angle(:,4);
leg2_theta2 = angle(:,5);
leg2_theta3 = angle(:,6);
subplot(332);
plot(t,leg2_theta1,'r',t,leg2_theta2,'b',t,leg2_theta3,'g');

leg3_theta1 = angle(:,7);
leg3_theta2 = angle(:,8);
leg3_theta3 = angle(:,9);
subplot(334);
plot(t,leg3_theta1,'r',t,leg3_theta2,'b',t,leg3_theta3,'g');

leg4_theta1 = angle(:,10);
leg4_theta2 = angle(:,11);
leg4_theta3 = angle(:,12);
subplot(335);
plot(t,leg4_theta1,'r',t,leg4_theta2,'b',t,leg4_theta3,'g');

%% 输出足尖末端曲线
x1 = angle(:,14);
y1 = angle(:,15);
z1 = angle(:,16);
x2 = angle(:,17);
y2 = angle(:,18);
z2 = angle(:,19);
x3 = angle(:,20);
y3 = angle(:,21);
z3 = angle(:,22);
x4 = angle(:,23);
y4 = angle(:,24);
z4 = angle(:,25);
x = angle(:,26);
y = angle(:,27);
z = angle(:,28);

subplot(333);
plot(t,x1,'r',t,x2,'b',t,x3,'y',t,x4,'m',t,x,'k');

subplot(339);
plot(t,y1,'r',t,y2,'b',t,y3,'y',t,y4,'m',t,y,'k');
 
subplot(336);
plot(t,z1,'r',t,z2,'b',t,z3,'y',t,z4,'m',t,z,'k');


