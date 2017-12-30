clear all
clc
syms x S1 S2 S3
t=[0,1,2,3,4,5];
q1=[0,0.3927,0.7854,1.0210,1.1781,1.5708];
q11=[0,0.1963,0.2749,0.3142,0.1963,0];
q2=[0,0.3236,0.6155,0.8472,0.9959,1.0472];
q22=[0,0.3466,0.2948,0.2142,0.1126,0];
q3=[0,0.1047,0.2094,0.3142,0.4189,0.5236];
q33=[0,0.3466,0.2948,0.2142,0.1126,0];

%三次样条插值函数（没有按照题目要求用spline命令是因为spline命令不是在算三次样条插值，所以自己写了一个函数）
for i=1:5
    T(i)=t(i+1)-t(i);
    delta1(i)=q1(i+1)-q1(i);
    delta2(i)=q2(i+1)-q2(i);
    delta3(i)=q3(i+1)-q3(i);
end
A=[2*T(1),T(1),0,0,0,0;T(1),2*(T(1)+T(2)),T(2),0,0,0;0,T(2),2*(T(2)+T(3)),T(3),0,0;0,0,T(3),2*(T(3)+T(4)),T(4),0;0,0,0,T(4),2*(T(4)+T(5)),T(5);0,0,0,0,T(5),2*T(5)];
z1=3*[delta1(1)/T(1);delta1(2)/T(2)-delta1(1)/T(1);delta1(3)/T(3)-delta1(2)/T(2);delta1(4)/T(4)-delta1(3)/T(3);delta1(5)/T(5)-delta1(4)/T(4);-delta1(5)/T(5)];
c1=inv(A)*z1;
z2=3*[delta2(1)/T(1);delta2(2)/T(2)-delta2(1)/T(1);delta2(3)/T(3)-delta2(2)/T(2);delta2(4)/T(4)-delta2(3)/T(3);delta2(5)/T(5)-delta2(4)/T(4);-delta2(5)/T(5)];
c2=inv(A)*z2;
z3=3*[delta3(1)/T(1);delta3(2)/T(2)-delta3(1)/T(1);delta3(3)/T(3)-delta3(2)/T(2);delta3(4)/T(4)-delta3(3)/T(3);delta3(5)/T(5)-delta3(4)/T(4);-delta3(5)/T(5)];
c3=inv(A)*z3;
for i=1:5 
    d1(i)=1/(3*T(i))*(c1(i+1)-c1(i));
    b1(i)=delta1(i)/T(i)-2/3*T(i)*c1(i)-T(i)/3*c1(i+1);
    S1(i)=q1(i)+b1(i)*(x-t(i))+c1(i)*(x-t(i))^2+d1(i)*(x-t(i))^3;
    
    d2(i)=1/(3*T(i))*(c2(i+1)-c2(i));
    b2(i)=delta2(i)/T(i)-2/3*T(i)*c2(i)-T(i)/3*c2(i+1);
    S2(i)=q2(i)+b2(i)*(x-t(i))+c2(i)*(x-t(i))^2+d2(i)*(x-t(i))^3;
    
    d3(i)=1/(3*T(i))*(c3(i+1)-c3(i));
    b3(i)=delta3(i)/T(i)-2/3*T(i)*c3(i)-T(i)/3*c3(i+1);
    S3(i)=q3(i)+b3(i)*(x-t(i))+c3(i)*(x-t(i))^2+d3(i)*(x-t(i))^3;
end
%作图
figure;
subplot(2,2,1);
for i=1:5
    e1=ezplot(S1(i),[t(i),t(i+1)]);
    hold on;
end
xlim([0,5]);
ylim([0,2]);
title('Poistion1');
xlabel('时间');
ylabel('角度');

subplot(2,2,2);
for i=1:5
    e2=ezplot(S2(i),[t(i),t(i+1)]);
    hold on;
end
xlim([0,5]);
ylim([0,1.5]);
title('Poistion2');
xlabel('时间');
ylabel('角度');

subplot(2,2,3);
for i=1:5
    e3=ezplot(S3(i),[t(i),t(i+1)]);
    hold on;
end
xlim([0,5]);
ylim([0,1]);
title('Poistion3');
xlabel('时间');
ylabel('角度');

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
%t=2.5s正向运动学以及雅可比矩阵
n1=sym2poly(subs(S1(3),x,2.5));
n2=sym2poly(subs(S2(3),x,2.5));
n3=sym2poly(subs(S3(3),x,2.5));
Forward=three_link.fkine([n1,n2,n3]);%正向运动学
J0=three_link.jacob0([n1,n2,n3]);%雅可比矩阵
disp('2.5s正向运动学答案为：');
disp(Forward);
disp('2.5s雅可比矩阵为：');
disp(J0);
