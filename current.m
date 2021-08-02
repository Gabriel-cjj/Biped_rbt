%% 检测电流
clear;
clc;

%% 不同运动顺序测试
% 读取参数测试顺序准备->前进->后退->左移->右移
A=readmatrix('actual_current on air.txt'); 
B=readmatrix('actual_current stand.txt');

current1=[A];
current2=[B];

%writematrix(angle(:,1:13),'T_angle.txt','Delimiter','tab');

 t=0.001:0.001:size(angle(:,1))/1000; %1ms执行一次

%% 输出空中电流
leg1_current1 = current1(:,5);
leg1_current2 = current1(:,4);
leg1_current3 = current1(:,3);
leg1_current4 = current1(:,2);
leg1_current5 = current1(:,1);
leg2_current1 = current1(:,6);
leg2_current2 = current1(:,7);
leg2_current3 = current1(:,8);
leg2_current4 = current1(:,9);
leg2_current5 = current1(:,10);

subplot(5,5,1)
plot(t,leg1_current1,'r');

subplot(5,5,2)
plot(t,leg1_current2,'r');

subplot(5,5,3)
plot(t,leg1_current3,'g');

subplot(5,5,4)
plot(t,leg1_current4,'k');

subplot(5,5,5)
plot(t,leg1_current5,'c');

subplot(5,5,6);
plot(t,leg2_current1,'r');

subplot(5,5,7);
plot(t,leg2_current2,'b');

subplot(5,5,8);
plot(t,leg2_current3,'g');

subplot(5,5,9);
plot(t,leg2_current4,'k');

subplot(5,5,10);
plot(t,leg2_current5,'c');


%% 输出站立电流
leg1_current1 = current2(:,5);
leg1_current2 = current2(:,4);
leg1_current3 = current2(:,3);
leg1_current4 = current2(:,2);
leg1_current5 = current2(:,1);
leg2_current1 = current2(:,6);
leg2_current2 = current2(:,7);
leg2_current3 = current2(:,8);
leg2_current4 = current2(:,9);
leg2_current5 = current2(:,10);

subplot(5,5,16)
plot(t,leg1_current1,'r');

subplot(5,5,17)
plot(t,leg1_current2,'r');

subplot(5,5,18)
plot(t,leg1_current3,'g');

subplot(5,5,19)
plot(t,leg1_current4,'k');

subplot(5,5,20)
plot(t,leg1_current5,'c');

subplot(5,5,21);
plot(t,leg2_current1,'r');

subplot(5,5,22);
plot(t,leg2_current2,'b');

subplot(5,5,23);
plot(t,leg2_current3,'g');

subplot(5,5,24);
plot(t,leg2_current4,'k');

subplot(5,5,25);
plot(t,leg2_current5,'c');




