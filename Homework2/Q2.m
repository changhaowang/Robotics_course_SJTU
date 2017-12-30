clear all %这个M文件是用于求解第二道题(3),(5)问的
clc
% syms theta1 theta2 theta3 d1 a2 a3 %如果需要得到表达式就用这段程序
theta1=pi/6; %如果需要计算具体数值就用这段程序
theta2=pi/6; 
theta3=pi/6;
d1=1;
a2=2;
a3=1;
A1=[cos(theta1),-sin(theta1),0,0;sin(theta1),cos(theta1),0,0;0,0,1,d1;0,0,0,1];
A2=[1,0,0,0;0,0,-1,0;0,1,0,0;0,0,0,1];
A=A1*A2;
B1=[cos(theta2),-sin(theta2),0,0;sin(theta2),cos(theta2),0,0;0,0,1,0;0,0,0,1];
B2=[1,0,0,a2;0,1,0,0;0,0,1,0;0,0,0,1];
B=B1*B2;
C1=[cos(theta3),-sin(theta3),0,0;sin(theta3),cos(theta3),0,0;0,0,1,0;0,0,0,1];
C2=[1,0,0,a3;0,1,0,0;0,0,1,0;0,0,0,1];
C=C1*C2;
X=A*B*C %正向运动学T31矩阵
R1=A(1:3,1:3);
R2=B(1:3,1:3);
R3=C(1:3,1:3);
z0=[0,0,1].';
z1=R1*z0;
z2=R1*R2*z0;
z3=R1*R2*R3*z0;
p0=[0,0,0].';
p1=A(1:3,4);
D=A*B;
p2=D(1:3,4);
p3=X(1:3,4);
J=[cross(z0,(p3-p0)),cross(z1,(p3-p1)),cross(z2,(p3-p2));z0,z1,z2] %几何雅可比矩阵