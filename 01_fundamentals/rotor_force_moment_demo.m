clear;
clc;

kf=1e-5;
km=1e-7;
w=[420;sqrt(2*400^2 - 420^2);420;sqrt(2*400^2 - 420^2)];

T=kf*w.^2;
Tt=sum(T);

Q=km*w.^2;

disp("four rotor thrusts:");
disp(T');
disp("total thrusts:");
disp(Tt);

disp("Four rotor reaction torques:");
disp(Q');

l=0.2;
tau_y=l*(T(1)-T(3));
tau_x=l*(T(4)-T(2));
tau_z=Q(1)+Q(3)-Q(2)-Q(4);

disp("Total thrust:");
disp(Tt);

disp("Roll torque:");
disp(tau_x);

disp("Pitch torque:");
disp(tau_y);

disp("Yaw torque:");
disp(tau_z);