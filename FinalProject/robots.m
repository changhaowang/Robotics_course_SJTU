
%编写人：王昌浩（正向运动学、运动空间求解、轨迹规划、逆向运动学、动力学、机器人示教）

clear all
clc
a1=1.50;alpha1=-pi/2;d1=4.50;theta1=pi/6;%（定义世界坐标系在底座处）
a2=6.00; alpha2=0; d2 =0; theta2=-pi/2;
a3=1.5;alpha3 =pi/2; d3 = 0; theta3=0;
a4=0;alpha4=-pi/2;d4=-6.55;theta4=-pi/2;%修改了theta4正负
a5=0; alpha5=-pi/2; d5 =0; theta5=pi/6;
a6=0;alpha6 =0; d6 =1.00; theta6=0;
 m1=29;m2=5.2;m3=21;m4=6;m5=1;m6=0.26;
 I1=[0.39 0 0;0 0.376 0;0 0 0.189];r1=[-2,0,0];
 I2=[0.114 0 0;0 0.112 0;0 0 0.006];r2=[-1.5,0,0];
 I3=[0.224 0 0;0 0.228 0;0 0 0.103];r3=[-1,0,0];
 I4=[0.05 0 0;0 0.05 0;0 0 0.008];
 I5=[0.001 0 0;0 0.001 0;0 0 0.0001];
 I6=[134*10^(-6) 0 0;0 99*10^(-6) 0;0 0 99*10^(-6)];
L(1) = Link([theta1, d1, a1, alpha1, 0]);
L(2) = Link([theta2, d2, a2, alpha2, 0]);
L(3) = Link([theta3, d3, a3, alpha3, 0]);
L(4) = Link([theta4, d4, a4, alpha4, 0]);
L(5) = Link([theta5, d5, a5, alpha5, 0]);
L(6) = Link([theta6, d6, a6, alpha6, 0]);
six_link=SerialLink(L, 'name', 'Fanuc');
TT =six_link.fkine([theta1 theta2 theta3  theta4 theta5 theta6]);%求正运动学解
J1 = six_link.jacob0([theta1 theta2 theta3  theta4 theta5 theta6]);%求雅可比矩阵
six_link.display;
six_link.teach();
a=six_link.getpos();
% six_link.plot([theta1 theta2 theta3  theta4 theta5 theta6]);%画图
%--------------------运动空间求解--------------------------------------
%该部分计算量巨大，运算时间很长
figure;
for theta1=-pi:0.5:pi
    for theta2=-pi:0.5:pi
        for theta3=-pi:0.5:pi
            for theta4=-pi:0.5:pi
                for theta5=-pi:0.5:pi
                    for theta6=-pi:0.5:pi
                        TT =six_link.fkine([theta1 theta2 theta3  theta4 theta5 theta6]);%求正运动学解
                        p=TT(1:3,4);
                        plot3(p(1),p(2),p(3),'o','Color','r');
                        hold on;
                    end
                end
            end
        end
    end
end
title('末端执行器运动空间','FontName','微软雅黑','FontSize',12);
%---------逆向运动学------------------------------------------------
%逆向运动学(在已知上述TT矩阵所表示的末端执行器的位姿情况下反解各个关节变量)
q0=[0 -pi/2 0 -pi/2 0 0];
Mask =[1 1 0 0 0 0];
qi=six_link.ikine(TT,q0,Mask);
% six_link.plot([qi2(1) qi2(2) qi2(3)  qi2(4) qi2(5) qi2(6)]);%画图
%轨迹规划
%目标：从世界坐标系原点开始做直线运动
T1=TT;
T2=TT*transl(20,30,20)*trotx(pi/3)*troty(pi/6)*trotz(pi/3);
qi1=six_link.ikine(T1,q0,Mask);
qi2=six_link.ikine(T2,q0,Mask);
n=10;%指定时间点数
t =linspace(1,10,n);
[q,qd,qdd]=jtraj(qi1,qi2,t);
figure;%绘制每一个关节位移——时间曲线图
e1(1)=plot(t,q(:,1),'Marker','*');
hold on;
e1(2)=plot(t,q(:,2),'r','Marker','o');
hold on;
e1(3)=plot(t,q(:,3),'g','Marker','+');
hold on;
e1(4)=plot(t,q(:,4),'y','Marker','s');
hold on;
e1(5)=plot(t,q(:,5),'k','Marker','.');
hold on;
e1(6)=plot(t,q(:,6),'c','Marker','^');
legend([e1(1),e1(2),e1(3),e1(4),e1(5),e1(6)],'关节1','关节2','关节3','关节4','关节5','关节6');
title('关节位移——时间曲线','FontName','微软雅黑','FontSize',12);
grid on;
figure;%绘制每一个关节速度——时间曲线图
e2(1)=plot(t,qd(:,1),'Marker','*');
hold on;
e2(2)=plot(t,qd(:,2),'r','Marker','o');
hold on;
e2(3)=plot(t,qd(:,3),'g','Marker','+');
hold on;
e2(4)=plot(t,qd(:,4),'y','Marker','s');
hold on;
e2(5)=plot(t,qd(:,5),'k','Marker','.');
hold on;
e2(6)=plot(t,qd(:,6),'c','Marker','^');
legend([e2(1),e2(2),e2(3),e2(4),e2(5),e2(6)],'关节1','关节2','关节3','关节4','关节5','关节6');
grid on;
title('关节速度——时间曲线','FontName','微软雅黑','FontSize',12);
figure;%每一个关节加速度——时间曲线图
e3(1)=plot(t,qdd(:,1),'Marker','*');
hold on;
e3(2)=plot(t,qdd(:,2),'r','Marker','o');
hold on;
e3(3)=plot(t,qdd(:,3),'g','Marker','+');
hold on;
e3(4)=plot(t,qdd(:,4),'y','Marker','s');
hold on;
e3(5)=plot(t,qdd(:,5),'k','Marker','.');
hold on;
e3(6)=plot(t,qdd(:,6),'c','Marker','^');
legend([e3(1),e3(2),e3(3),e3(4),e3(5),e3(6)],'关节1','关节2','关节3','关节4','关节5','关节6');
title('关节加速度——时间曲线','FontName','微软雅黑','FontSize',12);
grid on;


%动力学
L(1)=Link('d',d1,'a',a1,'alpha',alpha1,'m',m1,'I',I1);
L(2)=Link('d',d2,'a',a2,'alpha',alpha2,'m',m2,'I',I2);
L(3)=Link('d',d3,'a',a3,'alpha',alpha3,'m',m3,'I',I3);
L(4)=Link('d',d4,'a',a4,'alpha',alpha4,'m',m4,'I',I4);
L(5)=Link('d',d5,'a',a5,'alpha',alpha5,'m',m5,'I',I5);
L(6)=Link('d',d6,'a',a6,'alpha',alpha6,'m',m6,'I',I6);
six_link=SerialLink(L, 'name', 'Fanuc M-10iA/10M');
Q=0.008*six_link.rne(q,qd,qdd);%这里由于机器人建模时候以分米为单位建模，所以在这里换算了单位
%动力学作图
figure;
e4(1)=plot(t,Q(:,1),'Marker','*');
hold on;
e4(2)=plot(t,Q(:,2),'r','Marker','o');
hold on;
e4(3)=plot(t,Q(:,3),'g','Marker','+');
hold on;
e4(4)=plot(t,Q(:,4),'y','Marker','s');
hold on;
e4(5)=plot(t,Q(:,5),'k','Marker','.');
hold on;
e4(6)=plot(t,Q(:,6),'c','Marker','^');
xlabel('时间t/s','FontName','微软雅黑','FontSize',10);
ylabel('转矩N/m','FontName','微软雅黑','FontSize',10);
legend([e4(1),e4(2),e4(3),e4(4),e4(5),e4(6)],'关节1','关节2','关节3','关节4','关节5','关节6');
title('关节驱动力矩——时间曲线','FontName','微软雅黑','FontSize',12);
grid on;

%----------------运动学动画(画圆弧)--------------------------
%此段会保存出动画
fig=figure;
aviobj = avifile('robot.avi','compression','None');
for k=1:n
    six_link.plot(q(k,:));
    frame=getframe(fig);
    aviobj = addframe(aviobj,frame);
end
%机器人示教
six_link.plot(q0);%恢复原点
six_link.teach();
a=six_link.getpos();