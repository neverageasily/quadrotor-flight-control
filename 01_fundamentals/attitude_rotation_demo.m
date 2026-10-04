roll=10;
pitch=30;
yaw=45;

phi=deg2rad(roll);
theta=deg2rad(pitch);
psi=deg2rad(yaw);

Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];

Ry = [ cos(theta), 0, sin(theta);
       0,          1, 0;
      -sin(theta), 0, cos(theta)];

Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
      0,         0,        1];

R=Rz*Ry*Rx;

disp('Rotation Matrix R=');
disp(R);

check1=R'*R;
check2=det(R);
check3=R'-inv(R);

disp("R'*R");
disp(check1);
disp("det(R)=");
disp(check2);
disp("R'-inv(R)=");
disp(check3);

v_body=[1;0;0];
v_inertial=R*v_body;

disp("v_body=");
disp(v_body);
disp("v_inertial=");
disp(v_inertial);

figure;

quiver3(0,0,0, ...
    v_inertial(1), ...
    v_inertial(2), ...
    v_inertial(3), ...
    0, 'LineWidth', 2);

hold on;

% 惯性坐标系三个单位轴
quiver3(0,0,0,1,0,0,0);
quiver3(0,0,0,0,1,0,0);
quiver3(0,0,0,0,0,1,0);

grid on;
axis equal;

xlabel('X inertial');
ylabel('Y inertial');
zlabel('Z inertial');

xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);

title('Aircraft Heading in Inertial Frame');

view(3);