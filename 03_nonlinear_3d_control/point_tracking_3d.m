clc;
clear;

%% =========================
% Week 7 - 3D Cascade Control
% ==========================

%% 1. 飞机参数
m = 0.65;
g = 9.81;

Jx = 0.02;
Jy = 0.02;
Jz = 0.04;

J = [Jx 0  0;
     0  Jy 0;
     0  0  Jz];


%% 2. 仿真时间
dt = 0.01;
t_end = 10;
t = 0:dt:t_end;


%% 3. 目标位置
xd = 5;
yd = 3;
zd = 2;

psi_target = deg2rad(45);


%% 4. 位置外环 PD 参数
Kpx = 0.8;
Kdx = 1.4;

Kpy = 0.8;
Kdy = 1.4;

Kpz = 2.0;
Kdz = 1.8;


%% 5. 姿态内环 PD 参数
Kp_phi = 0.4;
Kd_phi = 0.15;

Kp_theta = 0.4;
Kd_theta = 0.15;

Kp_psi = 0.3;
Kd_psi = 0.12;


%% 6. 最大允许倾角
angle_max = deg2rad(20);


%% 7. 位置
x = zeros(size(t));
y = zeros(size(t));
z = zeros(size(t));


%% 8. 线速度
vx = zeros(size(t));
vy = zeros(size(t));
vz = zeros(size(t));


%% 9. 线加速度
ax = zeros(size(t));
ay = zeros(size(t));
az = zeros(size(t));


%% 10. 欧拉角
phi   = zeros(size(t));
theta = zeros(size(t));
psi   = zeros(size(t));


%% 11. 机体系角速度
p = zeros(size(t));
q = zeros(size(t));
r = zeros(size(t));


%% 12. 期望加速度
ax_d = zeros(size(t));
ay_d = zeros(size(t));
az_d = zeros(size(t));


%% 13. 期望姿态
phi_d   = zeros(size(t));
theta_d = zeros(size(t));
psi_d   = zeros(size(t));


%% 14. 控制输入
T = zeros(size(t));

tau_x = zeros(size(t));
tau_y = zeros(size(t));
tau_z = zeros(size(t));


%% =========================
% 15. 主循环
% ==========================

for k = 1:length(t)-1

    %% A. 位置外环
    ex = xd - x(k);
    ey = yd - y(k);
    ez = zd - z(k);

    ax_d(k) = Kpx*ex - Kdx*vx(k);

    ay_d(k) = Kpy*ey - Kdy*vy(k);

    az_d(k) = Kpz*ez - Kdz*vz(k);


   %% 期望加速度 -> 期望推力矢量

A_d = sqrt(ax_d(k)^2 ...
         + ay_d(k)^2 ...
         + (g + az_d(k))^2);

%% 总推力
T(k) = m * A_d;

%% 期望姿态
theta_d(k) = atan2(ax_d(k), ...
                   g + az_d(k));

phi_d(k) = -asin(ay_d(k) / A_d);

psi_d(k) = psi_target;


    %% E. 姿态内环

    e_phi   = phi_d(k)   - phi(k);
    e_theta = theta_d(k) - theta(k);
    e_psi   = psi_d(k)   - psi(k);

    tau_x(k) = Kp_phi*e_phi ...
             - Kd_phi*p(k);

    tau_y(k) = Kp_theta*e_theta ...
             - Kd_theta*q(k);

    tau_z(k) = Kp_psi*e_psi ...
             - Kd_psi*r(k);

    tau = [tau_x(k);
           tau_y(k);
           tau_z(k)];


    %% F. 3D 转动动力学

    omega = [p(k);
             q(k);
             r(k)];

    omega_dot = J \ ...
        (tau - cross(omega, J*omega));


    %% G. 角速度积分

    p(k+1) = p(k) + omega_dot(1)*dt;

    q(k+1) = q(k) + omega_dot(2)*dt;

    r(k+1) = r(k) + omega_dot(3)*dt;


    %% H. p,q,r -> 欧拉角变化率

    E = [1, sin(phi(k))*tan(theta(k)), ...
            cos(phi(k))*tan(theta(k));

         0, cos(phi(k)), ...
           -sin(phi(k));

         0, sin(phi(k))/cos(theta(k)), ...
            cos(phi(k))/cos(theta(k))];

    euler_dot = E * omega;


    %% I. 欧拉角积分

    phi(k+1) = phi(k) ...
             + euler_dot(1)*dt;

    theta(k+1) = theta(k) ...
               + euler_dot(2)*dt;

    psi(k+1) = psi(k) ...
             + euler_dot(3)*dt;


    %% J. Body -> Inertial 旋转矩阵

    Rx = [1 0 0;
          0 cos(phi(k)) -sin(phi(k));
          0 sin(phi(k))  cos(phi(k))];

    Ry = [ cos(theta(k)) 0 sin(theta(k));
           0              1 0;
          -sin(theta(k)) 0 cos(theta(k))];

    Rz = [cos(psi(k)) -sin(psi(k)) 0;
          sin(psi(k))  cos(psi(k)) 0;
          0            0           1];

    R = Rz * Ry * Rx;


    %% K. 机体系总推力

    F_body = [0;
              0;
              T(k)];


    %% L. 推力转换到惯性系

    F_inertial = R * F_body;


    %% M. 3D 平动动力学

    a = F_inertial/m + [0;
                        0;
                       -g];

    ax(k) = a(1);
    ay(k) = a(2);
    az(k) = a(3);


    %% N. 加速度 -> 速度

    vx(k+1) = vx(k) + ax(k)*dt;
    vy(k+1) = vy(k) + ay(k)*dt;
    vz(k+1) = vz(k) + az(k)*dt;


    %% O. 速度 -> 位置

    x(k+1) = x(k) + vx(k)*dt;
    y(k+1) = y(k) + vy(k)*dt;
    z(k+1) = z(k) + vz(k)*dt;

end


%% 16. 补最后一个点
ax(end) = ax(end-1);
ay(end) = ay(end-1);
az(end) = az(end-1);

ax_d(end) = ax_d(end-1);
ay_d(end) = ay_d(end-1);
az_d(end) = az_d(end-1);

phi_d(end) = phi_d(end-1);
theta_d(end) = theta_d(end-1);
psi_d(end) = psi_d(end-1);

T(end) = T(end-1);

tau_x(end) = tau_x(end-1);
tau_y(end) = tau_y(end-1);
tau_z(end) = tau_z(end-1);


%% 17. 位置响应

figure;

plot(t, x, 'LineWidth', 1.5);
hold on;
plot(t, y, 'LineWidth', 1.5);
plot(t, z, 'LineWidth', 1.5);

yline(xd, '--');
yline(yd, '--');
yline(zd, '--');

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

legend('x', 'y', 'z');

title('3D Position Response');


%% 18. Roll 跟踪

figure;

plot(t, rad2deg(phi), 'LineWidth', 1.5);
hold on;

plot(t, rad2deg(phi_d), '--', ...
     'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Roll Angle (deg)');

legend('\phi', '\phi_d');

title('Roll Tracking');


%% 19. Pitch 跟踪

figure;

plot(t, rad2deg(theta), 'LineWidth', 1.5);
hold on;

plot(t, rad2deg(theta_d), '--', ...
     'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Pitch Angle (deg)');

legend('\theta', '\theta_d');

title('Pitch Tracking');


%% 20. 3D 飞行轨迹

figure;

plot3(x, y, z, 'LineWidth', 1.5);

hold on;

plot3(xd, yd, zd, 'o', ...
      'MarkerSize', 8);

grid on;
axis equal;

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

legend('Trajectory', 'Target');

title('3D Flight Trajectory');

view(3);