function [ Z1,Y1,Z2 ] = Q3_ZYZ( R )
%ZYZ欧拉角
while abs(det(R)-1)>10^(-6)
    R=input('请重新输入旋转矩阵R=');
end
Z1=atan2(R(2,3),R(1,3));
Y1=atan2((R(1,3)^2+R(2,3)^2)^0.5,R(3,3));
Z2=atan2(R(3,2),-R(3,1));
end

