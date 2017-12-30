clear all
clc
syms x y y1 y11
t=[0,1,2,3,4,5];
q1=[0,0.3927,0.7854,1.0210,1.1781,1.5708];
q11=[0,0.1963,0.2749,0.3142,0.1963,0];
q2=[0,0.3236,0.6155,0.8472,0.9959,1.0472];
q22=[0,0.3466,0.2948,0.2142,0.1126,0];
q3=[0,0.1047,0.2094,0.3142,0.4189,0.5236];
q33=[0,0.3466,0.2948,0.2142,0.1126,0];

for i=1:5
   A=[1,t(i),t(i)^2,t(i)^3;1,t(i+1),t(i+1)^2,t(i+1)^3;0,1,2*t(i),3*t(i)^2;0,1,2*t(i+1),3*t(i+1)^2];
   b1=[q1(i);q1(i+1);q11(i);q11(i+1)];
   b2=[q2(i);q2(i+1);q22(i);q22(i+1)];
   b3=[q3(i);q3(i+1);q33(i);q33(i+1)];
   X1=inv(A)*b1;
   X2=inv(A)*b2;
   X3=inv(A)*b3;
   y1(i)=X1(4)*x^3+X1(3)*x^2+X1(2)*x+X1(1);
   %y11(i)=3*X1(4)*x^2+2*X1(3)*x+X1(2);%第一个关节速度拟合曲线
   %y111(i)=6*X1(4)*x+2*X1(3);%第一个关节加速度拟合曲线
   y2(i)=X2(4)*x^3+X2(3)*x^2+X2(2)*x+X2(1);
   %y22(i)=3*X2(4)*x^2+2*X2(3)*x+X2(2);%第二个关节速度拟合曲线
   %y222(i)=6*X2(4)*x+2*X2(3);%第二个关节加速度拟合曲线
   y3(i)=X3(4)*x^3+X3(3)*x^2+X3(2)*x+X3(1);
   %y33(i)=3*X3(4)*x^2+2*X3(3)*x+X3(2);%第三个关节速度拟合曲线
   %y333(i)=6*X3(4)*x+2*X3(3);%第三个关节加速度拟合曲线
end
%作图以及jtraj验证
%jtraj验证
[q01,qd1,qdd1]=jtraj(q1(1,1:5),q1(1,2:6),t);
q01=reshape(q01,[1,30]);
[q02,qd2,qdd2]=jtraj(q2(1,1:5),q2(1,2:6),t);
q02=reshape(q02,[1,30]);
[q03,qd3,qdd3]=jtraj(q3(1,1:5),q3(1,2:6),t);
q03=reshape(q03,[1,30]);
t1=linspace(0,5,30);
figure;
subplot(2,2,1);
for i=1:5
    e1=ezplot(y1(i),[t(i),t(i+1)]);
    hold on;
end
xlim([0,5]);
ylim([0,2]);
title('Poistion1');
xlabel('时间');
ylabel('角度');
e2=plot(t1,q01,'r');
legend([e1,e2],'多项式插值','jtraj');


subplot(2,2,2);
for i=1:5
    e3=ezplot(y2(i),[t(i),t(i+1)]);
    hold on;
end
xlim([0,5]);
ylim([0,1.5]);
title('Poistion2');
xlabel('时间');
ylabel('角度');
e4=plot(t1,q02,'r');
legend([e3,e4],'多项式插值','jtraj');

subplot(2,2,3);
for i=1:5
    e5=ezplot(y3(i),[t(i),t(i+1)]);
    hold on;
end
xlim([0,5]);
ylim([0,1]);
title('Poistion3');
xlabel('时间');
ylabel('角度');
e6=plot(t1,q03,'r');
legend([e5,e6],'多项式插值','jtraj');

a1=1;%定义a1的长度为1；
a2=1;%定义a2的长度为1；
a3=0.5;%定义a3的长度为0.5；
alpha1=0;
alpha2=0;
alpha3=0;
d1=0;
d2=0;
d3=0;
theta1=pi/6;%定义theta1为30°
theta2=pi/3;%定义theta2为60°
theta3=pi/3;%定义theta3为60°
L(1)=Link([theta1,d1,a1,alpha1,0]);
L(2)=Link([theta2,d2,a2,alpha2,0]);
L(3)=Link([theta3,d3,a3,alpha3,0]);
three_link=SerialLink(L,'name','ThreeLink');
subplot(2,2,4);
three_link.plot([pi/6,pi/3,pi/3]);
%计算2.5s运动学方程以及雅可比矩阵
n1=sym2poly(subs(y1(3),x,2.5));
n2=sym2poly(subs(y2(3),x,2.5));
n3=sym2poly(subs(y3(3),x,2.5));
Forward=three_link.fkine([n1,n2,n3]);%正向运动学
J0=three_link.jacob0([n1,n2,n3]);%雅可比矩阵
disp('2.5s正向运动学答案为：');
disp(Forward);
disp('2.5s雅可比矩阵为：');
disp(J0);

