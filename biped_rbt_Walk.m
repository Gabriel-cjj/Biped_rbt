%% ����walk��̬�ƶ��ĽǶ����ߣ����adamsʹ��
clear;
clc;

%% ��ͬ�˶�˳�����
% ��ȡ��������˳��׼��->ǰ��->����->����->����
A=readmatrix('walkStep.txt');  %׼��
% B=readmatrix('forward.txt');  %ǰ��
% C=readmatrix('back.txt');     %����
% D=readmatrix('left.txt');     %����
% E=readmatrix('right.txt');    %����
%F=readmatrix('tabu.txt');    %̤��
angle=[A];

%writematrix(angle(:,1:13),'T_angle.txt','Delimiter','tab');

 t=0.001:0.001:size(angle(:,1))/1000; %1msִ��һ��

%% ����ؽڽǶ�����
leg1_theta1 = angle(:,1);
leg1_theta2 = angle(:,2);
leg1_theta3 = angle(:,3);
leg1_theta4 = angle(:,4);
leg1_theta5 = angle(:,5);
leg2_theta1 = angle(:,6);
leg2_theta2 = angle(:,7);
leg2_theta3 = angle(:,8);
leg2_theta4 = angle(:,9);
leg2_theta5 = angle(:,10);

subplot(331)
plot(t,leg1_theta1,'r',t,leg1_theta2,'b',t,leg1_theta3,'g',t,leg1_theta4,'k',t,leg1_theta5,'c');

leg4_theta1 = angle(:,10);

subplot(332);
plot(t,leg2_theta1,'r',t,leg2_theta2,'b',t,leg2_theta3,'g',t,leg2_theta4,'k',t,leg2_theta5,'c');

%% ������ĩ������
x1 = angle(:,12);
y1 = angle(:,13);
z1 = angle(:,14);
x2 = angle(:,15);
y2 = angle(:,16);
z2 = angle(:,17);

x = angle(:,18);
y = angle(:,19);
z = angle(:,20);

subplot(333);
plot(t,x1,'r',t,x2,'b',t,x,'y');

subplot(336);
plot(t,y1,'r',t,y2,'b',t,y,'y');


subplot(339);
plot(t,z1,'r',t,z2,'b',t,z,'y');




