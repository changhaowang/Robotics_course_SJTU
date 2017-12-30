function [ T ] = ZYZ( Z1,Y1,Z2,o )

Z1=Z1/180*pi;
Y1=Y1/180*pi;
Z2=Z2/180*pi;
o=o.';
Rz1=[cos(Z1),-sin(Z1),0;sin(Z1),cos(Z1),0;0,0,1];
Ry1=[cos(Y1),0,sin(Y1);0,1,0;-sin(Y1),0,cos(Y1)];
Rz2=[cos(Z2),-sin(Z2),0;sin(Z2),cos(Z2),0;0,0,1];
R=Rz1*Ry1*Rz2;
T=[R,o.';zeros(1,3),1];
end

