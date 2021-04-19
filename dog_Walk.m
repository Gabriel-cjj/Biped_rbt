%% ����walk��̬�ƶ��ĽǶ����ߣ����adamsʹ��
clear;
clc;

%% ��ͬ�˶�˳�����
% ��ȡ��������˳��׼��->ǰ��->����->����->����
A=readmatrix('prepare.txt');  %׼��
B=readmatrix('forward.txt');  %ǰ��
C=readmatrix('back.txt');     %����
D=readmatrix('left.txt');     %����
E=readmatrix('right.txt');    %����
%F=readmatrix('tabu.txt');    %̤��
angle=[A;B;C;D;E];

writematrix(angle(:,1:13),'T_angle.txt','Delimiter','tab');
t=angle(:,13);
 %t=0.001:0.001:size(angle(:,1))/1000; %1msִ��һ��

%% ����ؽڽǶ�����
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

%% ������ĩ������
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


