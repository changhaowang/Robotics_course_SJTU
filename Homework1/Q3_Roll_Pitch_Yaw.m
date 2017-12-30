function [ Roll,Pitch,Yaw ] = Q3_Roll_Pitch_Yaw( R )
%Roll_Pitch_Yaw
while abs(det(R)-1)>10^(-6)
    R=input('请重新输入旋转矩阵R=');
end
Roll=atan2(R(3,2),R(3,3));
Pitch=atan2(-R(3,1),(R(3,2)^2+R(3,3)^2)^0.5);
Yaw=atan2(R(2,1),R(1,1));
end

