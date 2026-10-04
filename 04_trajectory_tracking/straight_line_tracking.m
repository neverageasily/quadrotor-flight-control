clc;
clear;
close all;

% Straight-line trajectory tracking with cascaded position/attitude control.

%% =========================
%  1. 基本参数
%  =========================

m = 0.65;              % 无人机质量 kg
g = 9.81;              % 重力加速度 m/s^2

Jx = 0.02;
Jy = 0.02;
Jz = 0.04;

J = diag([Jx Jy Jz]);  % 转动惯量矩阵


%% =========================
%  2. 仿真时间
%  =========================

dt = 0.01;
t_end = 10;

t = 0:dt:t_end;
N = length(t);


%% =========================
%  3. 期望轨迹
%  =========================

% x方向匀速运动
xd = 0.5 * t;

% y方向不动
yd = 0;

% 高度保持2 m
zd = 2;

% 期望速度
vxd = 0.5;
vyd = 0;
vzd = 0;


%% =========================
%  4. 位置外环增益
%  =========================

Kpx = 0.8;
Kdx = 1.4;

Kpy = 0.8;
Kdy = 1.4;

Kpz = 2.0;
Kdz = 1.8;


%% =========================
%  5. 姿态内环增益
%  =========================

Kp_phi = 0.4;
Kd_phi = 0.15;

Kp_theta = 0.4;
Kd_theta = 0.15;

Kp_psi = 0.3;
Kd_psi = 0.12;


%% =========================
%  6. 初始化状态
%  =========================

% 位置
x = zeros(size(t));
y = zeros(size(t));
z = zeros(size(t));

% 速度
vx = zeros(size(t));
vy = zeros(size(t));
vz = zeros(size(t));

% 欧拉角
phi   = zeros(size(t));
theta = zeros(size(t));
psi   = zeros(size(t));

% 机体系角速度
p = zeros(size(t));
q = zeros(size(t));
r = zeros(size(t));


%% =========================
%  7. 初始化控制变量
%  =========================

% 外环给出的 commanded acceleration
ax_c = zeros(size(t));
ay_c = zeros(size(t));
az_c = zeros(size(t));

% 期望姿态
phi_d   = zeros(size(t));
theta_d = zeros(size(t));
psi_d   = zeros(size(t));

% 总推力
T = zeros(size(t));

% 控制力矩
tau_x = zeros(size(t));
tau_y = zeros(size(t));
tau_z = zeros(size(t));


%% =========================
%  8. 姿态角限制
%  =========================

angle_max = deg2rad(20);


%% =========================
%  9. 主循环
%  =========================

for k = 1:N-1

    %% --------------------------------
    %  A. 位置误差
    %  ---------------------------------

    ex = xd(k) - x(k);
    ey = yd    - y(k);
    ez = zd    - z(k);


    %% --------------------------------
    %  B. 速度误差
    %  ---------------------------------

    evx = vxd - vx(k);
    evy = vyd - vy(k);
    evz = vzd - vz(k);


    %% --------------------------------
    %  C. 位置外环
    %
    %  位置误差 + 速度误差
    %  -> commanded acceleration
    %  ---------------------------------

    ax_c(k) = Kpx*ex + Kdx*evx;

    ay_c(k) = Kpy*ey + Kdy*evy;

    az_c(k) = Kpz*ez + Kdz*evz;


    %% --------------------------------
    %  D. commanded acceleration
    %     -> 期望推力矢量
    %  ---------------------------------

    A_c = sqrt( ax_c(k)^2 ...
              + ay_c(k)^2 ...
              + (g + az_c(k))^2 );


    %% --------------------------------
    %  E. 总推力
    %  ---------------------------------

    T(k) = m * A_c;


    %% --------------------------------
    %  F. 推力矢量 -> 期望姿态
    %
    %  当前仍然假设 psi_d = 0
    %  ---------------------------------

    theta_d(k) = atan2( ax_c(k), ...
                        g + az_c(k) );

    phi_d(k) = -asin( ay_c(k) / A_c );

    psi_d(k) = 0;


    %% --------------------------------
    %  G. 限制期望roll / pitch
    %  ---------------------------------

    theta_d(k) = max(min(theta_d(k), angle_max), ...
                     -angle_max);

    phi_d(k) = max(min(phi_d(k), angle_max), ...
                   -angle_max);


    %% --------------------------------
    %  H. 姿态误差
    %  ---------------------------------

    e_phi   = phi_d(k)   - phi(k);
    e_theta = theta_d(k) - theta(k);
    e_psi   = psi_d(k)   - psi(k);


    %% --------------------------------
    %  I. 姿态内环 PD
    %
    %  期望姿态 -> 控制力矩
    %  ---------------------------------

    tau_x(k) = Kp_phi * e_phi ...
             - Kd_phi * p(k);

    tau_y(k) = Kp_theta * e_theta ...
             - Kd_theta * q(k);

    tau_z(k) = Kp_psi * e_psi ...
             - Kd_psi * r(k);


    %% =================================
    %  下面开始进入 Week 6 的
    %  无人机动力学模型
    %  =================================


    %% --------------------------------
    %  J. 当前角速度
    %  ---------------------------------

    omega = [p(k);
             q(k);
             r(k)];


    %% --------------------------------
    %  K. 当前控制力矩
    %  ---------------------------------

    tau = [tau_x(k);
           tau_y(k);
           tau_z(k)];


    %% --------------------------------
    %  L. 刚体旋转动力学
    %
    %  J*omega_dot
    %  + omega × (J*omega)
    %  = tau
    %  ---------------------------------

    omega_dot = J \ ...
        (tau - cross(omega, J*omega));


    %% --------------------------------
    %  M. 更新角速度
    %  ---------------------------------

    p(k+1) = p(k) + omega_dot(1)*dt;
    q(k+1) = q(k) + omega_dot(2)*dt;
    r(k+1) = r(k) + omega_dot(3)*dt;


    %% --------------------------------
    %  N. Euler角运动学矩阵 E
    %  ---------------------------------

    phi_k   = phi(k);
    theta_k = theta(k);

    E = [1, ...
         sin(phi_k)*tan(theta_k), ...
         cos(phi_k)*tan(theta_k);

         0, ...
         cos(phi_k), ...
        -sin(phi_k);

         0, ...
         sin(phi_k)/cos(theta_k), ...
         cos(phi_k)/cos(theta_k)];


    %% --------------------------------
    %  O. Euler角速度
    %  ---------------------------------

    euler_dot = E * omega;


    %% --------------------------------
    %  P. 更新Euler角
    %  ---------------------------------

    phi(k+1) = phi(k) ...
             + euler_dot(1)*dt;

    theta(k+1) = theta(k) ...
               + euler_dot(2)*dt;

    psi(k+1) = psi(k) ...
             + euler_dot(3)*dt;


    %% --------------------------------
    %  Q. 构造旋转矩阵
    %
    %  R = Rz * Ry * Rx
    %
    %  Body -> Inertial
    %  ---------------------------------

    Rx = [1, 0, 0;
          0, cos(phi(k)), -sin(phi(k));
          0, sin(phi(k)),  cos(phi(k))];

    Ry = [ cos(theta(k)), 0, sin(theta(k));
           0,             1, 0;
          -sin(theta(k)), 0, cos(theta(k))];

    Rz = [cos(psi(k)), -sin(psi(k)), 0;
          sin(psi(k)),  cos(psi(k)), 0;
          0,            0,           1];

    R = Rz * Ry * Rx;


    %% --------------------------------
    %  R. 机体系推力
    %  ---------------------------------

    F_body = [0;
              0;
              T(k)];


    %% --------------------------------
    %  S. 转到惯性坐标系
    %  ---------------------------------

    F_inertial = R * F_body;


    %% --------------------------------
    %  T. 平移动力学
    %
    %  a = F/m + gravity
    %  ---------------------------------

    a = F_inertial / m ...
      + [0;
         0;
        -g];


    %% --------------------------------
    %  U. 更新速度
    %  ---------------------------------

    vx(k+1) = vx(k) + a(1)*dt;
    vy(k+1) = vy(k) + a(2)*dt;
    vz(k+1) = vz(k) + a(3)*dt;


    %% --------------------------------
    %  V. 更新位置
    %  ---------------------------------

    x(k+1) = x(k) + vx(k)*dt;
    y(k+1) = y(k) + vy(k)*dt;
    z(k+1) = z(k) + vz(k)*dt;

end


%% =========================
%  10. 补齐最后一个采样点
%  =========================

ax_c(end) = ax_c(end-1);
ay_c(end) = ay_c(end-1);
az_c(end) = az_c(end-1);

phi_d(end)   = phi_d(end-1);
theta_d(end) = theta_d(end-1);
psi_d(end)   = psi_d(end-1);

T(end) = T(end-1);

tau_x(end) = tau_x(end-1);
tau_y(end) = tau_y(end-1);
tau_z(end) = tau_z(end-1);


%% =========================
%  11. Figure 1
%  x轨迹跟踪
%  =========================

figure;

plot(t, xd, '--', 'LineWidth', 1.5);
hold on;

plot(t, x, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('x (m)');

legend('x_d', 'x');

title('X Trajectory Tracking');

grid on;


%% =========================
%  12. Figure 2
%  三个位置
%  =========================

figure;

plot(t, x, 'LineWidth', 1.5);
hold on;

plot(t, y, 'LineWidth', 1.5);
plot(t, z, 'LineWidth', 1.5);

plot(t, xd, '--', 'LineWidth', 1.2);

xlabel('Time (s)');
ylabel('Position (m)');

legend('x', 'y', 'z', 'x_d');

title('Position Response');

grid on;


%% =========================
%  13. Figure 3
%  Pitch跟踪
%  =========================

figure;

plot(t, rad2deg(theta), ...
    'LineWidth', 1.5);

hold on;

plot(t, rad2deg(theta_d), '--', ...
    'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Pitch angle (deg)');

legend('\theta', '\theta_d');

title('Pitch Tracking');

grid on;


%% =========================
%  14. Figure 4
%  Roll跟踪
%  =========================

figure;

plot(t, rad2deg(phi), ...
    'LineWidth', 1.5);

hold on;

plot(t, rad2deg(phi_d), '--', ...
    'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Roll angle (deg)');

legend('\phi', '\phi_d');

title('Roll Tracking');

grid on;


%% =========================
%  15. Figure 5
%  三维飞行轨迹
%  =========================

figure;

plot3(x, y, z, ...
    'LineWidth', 1.8);

hold on;

plot3(xd, ...
      zeros(size(t)), ...
      2*ones(size(t)), ...
      '--', ...
      'LineWidth', 1.5);

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

legend('Actual trajectory', ...
       'Desired trajectory');

title('3D Trajectory Tracking');

grid on;
axis equal;
view(3);


%% =========================
%  16. Figure 6
%  x方向速度跟踪
%  =========================

figure;

plot(t, vx, ...
    'LineWidth', 1.5);

hold on;

plot(t, ...
    vxd*ones(size(t)), ...
    '--', ...
    'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Velocity (m/s)');

legend('v_x', 'v_{x,d}');

title('X Velocity Tracking');

grid on;
