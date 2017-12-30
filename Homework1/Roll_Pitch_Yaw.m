function [ T ] = Roll_Pitch_Yaw( Roll,Pitch,Yaw,o )

Roll=Roll/180*pi;
Pitch=Pitch/180*pi;
Yaw=Yaw/180*pi;
o=o.';
Ryaw=[cos(Yaw),-sin(Yaw),0;sin(Yaw),cos(Yaw),0;0,0,1];
Rpitch=[cos(Pitch),0,sin(Pitch);0,1,0;-sin(Pitch),0,cos(Pitch)];
Rroll=[1,0,0;0,cos(Roll),-sin(Roll);0,sin(Roll),cos(Roll)];
R=Ryaw*Rpitch*Rroll;
T=[R,o.';zeros(1,3),1];
end

