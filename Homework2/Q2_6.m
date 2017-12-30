clear all
clc
a1=0;% 将a1设置为1
alpha1=pi/2;
d1=1;% 将d1设置为1
theta1=pi/6; %将theta1设置为pi/6
a2=2;% 将a2设置为1
alpha2=0;
d2=0;
theta2=pi/6;% 将转动角度theta2设置为pi/6
a3=1;% 将a3设置为1
alpha3=0;
d3=0;
theta3=pi/6;%将转动角度theta3设置为pi/6
L(1)=Link([theta1,d1,a1,alpha1,0]);
L(2)=Link([theta2,d2,a2,alpha2,0]);
L(3)=Link([theta3,d3,a3,alpha2,0]);
three_link=SerialLink(L,'name','ThreeLink');
T=three_link.fkine([theta1 theta2 theta3]) %正向运动学求解
q0=[theta1 theta2 theta3];
Mask=[1 1 0 0 0 0];
qi=three_link.ikine(T,q0, Mask)  %逆向运动学求解
three_link.plot([theta1 theta2 theta3])    %画图
J0 = three_link.jacob0([theta1 theta2 theta3])% 几何雅可比矩阵求解