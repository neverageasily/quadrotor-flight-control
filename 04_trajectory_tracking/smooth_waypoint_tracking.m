clc;
clear;
close all;


%% =========================================================
%  Smooth Waypoint Tracking with Quintic Polynomial Segments
%
%  Waypoints:
%
%  P0 = (0,0)
%  P1 = (2,0)
%  P2 = (2,2)
%  P3 = (4,2)
%
%  使用一般五次多项式
%  中间waypoint具有非零速度
% ==========================================================


%% =========================
%  1. 无人机参数
% ==========================

m = 0.65;
g = 9.81;

Jx = 0.02;
Jy = 0.02;
Jz = 0.04;

J = diag([Jx Jy Jz]);


%% =========================
%  2. 仿真时间
% ==========================

dt = 0.01;

Tseg = 4;      % 每个segment持续4秒


%% =========================
%  3. Waypoints
% ==========================

waypoints = [
    0 0;
    2 0;
    2 2;
    4 2
];

num_wp = size(waypoints,1);

num_seg = num_wp - 1;

t_end = num_seg * Tseg;

t = 0:dt:t_end;

N = length(t);


%% =========================
%  4. 每个Waypoint的速度
% ==========================

% 一开始全部设为0
v_wp = zeros(size(waypoints));


% 中间waypoint自动计算速度
%
% v_i =
% (P_(i+1) - P_(i-1))
% / (2*Tseg)

for i = 2:num_wp-1

    v_wp(i,:) = ...
        (waypoints(i+1,:) ...
        - waypoints(i-1,:)) ...
        / (2*Tseg);

end


% 显示一下每个waypoint的速度
disp('Waypoint velocities:');
disp(v_wp);


%% =========================
%  5. 每个Waypoint的加速度
% ==========================

% 目前全部规定为0

a_wp = zeros(size(waypoints));


%% =========================
%  6. 初始化期望轨迹
% ==========================

% 位置
xd = zeros(size(t));
yd = zeros(size(t));

% 速度
vxd = zeros(size(t));
vyd = zeros(size(t));

% 加速度
ax_d = zeros(size(t));
ay_d = zeros(size(t));


% 高度
zd = 2;

vzd = 0;

az_d = 0;


%% =========================
%  7. 五次多项式矩阵 M
% ==========================

T = Tseg;

M = [
    1  0    0      0       0        0;

    0  1    0      0       0        0;

    0  0    2      0       0        0;

    1  T    T^2    T^3     T^4      T^5;

    0  1    2*T    3*T^2   4*T^3    5*T^4;

    0  0    2      6*T     12*T^2   20*T^3
];


%% =========================================================
%  8. 自动生成平滑期望轨迹
% ==========================================================

for k = 1:N

    %% -------------------------
    % 当前属于第几个segment
    % --------------------------

    seg = min( ...
        floor(t(k)/Tseg) + 1, ...
        num_seg );


    %% -------------------------
    % 当前segment的局部时间
    % --------------------------

    tau = ...
        t(k) ...
        - (seg-1)*Tseg;


    %% -------------------------
    % 当前segment起点和终点
    % --------------------------

    P0 = waypoints(seg,:);

    Pf = waypoints(seg+1,:);


    %% -------------------------
    % 起点、终点速度
    % --------------------------

    V0 = v_wp(seg,:);

    Vf = v_wp(seg+1,:);


    %% -------------------------
    % 起点、终点加速度
    % --------------------------

    Acc0 = a_wp(seg,:);

    Accf = a_wp(seg+1,:);


    %% =========================
    % x方向边界条件
    %
    % [x0
    %  vx0
    %  ax0
    %  xf
    %  vxf
    %  axf]
    % ==========================

    bx = [
        P0(1);
        V0(1);
        Acc0(1);
        Pf(1);
        Vf(1);
        Accf(1)
    ];


    %% 解x方向五次多项式系数

    cx = M \ bx;


    %% =========================
    % y方向边界条件
    % ==========================

    by = [
        P0(2);
        V0(2);
        Acc0(2);
        Pf(2);
        Vf(2);
        Accf(2)
    ];


    %% 解y方向五次多项式系数

    cy = M \ by;


    %% =========================
    % 当前期望位置
    % ==========================

    xd(k) = ...
          cx(1) ...
        + cx(2)*tau ...
        + cx(3)*tau^2 ...
        + cx(4)*tau^3 ...
        + cx(5)*tau^4 ...
        + cx(6)*tau^5;


    yd(k) = ...
          cy(1) ...
        + cy(2)*tau ...
        + cy(3)*tau^2 ...
        + cy(4)*tau^3 ...
        + cy(5)*tau^4 ...
        + cy(6)*tau^5;


    %% =========================
    % 当前期望速度
    % ==========================

    vxd(k) = ...
          cx(2) ...
        + 2*cx(3)*tau ...
        + 3*cx(4)*tau^2 ...
        + 4*cx(5)*tau^3 ...
        + 5*cx(6)*tau^4;


    vyd(k) = ...
          cy(2) ...
        + 2*cy(3)*tau ...
        + 3*cy(4)*tau^2 ...
        + 4*cy(5)*tau^3 ...
        + 5*cy(6)*tau^4;


    %% =========================
    % 当前期望加速度
    % ==========================

    ax_d(k) = ...
          2*cx(3) ...
        + 6*cx(4)*tau ...
        + 12*cx(5)*tau^2 ...
        + 20*cx(6)*tau^3;


    ay_d(k) = ...
          2*cy(3) ...
        + 6*cy(4)*tau ...
        + 12*cy(5)*tau^2 ...
        + 20*cy(6)*tau^3;

end


%% =========================
%  9. 位置外环增益
% ==========================

Kpx = 0.8;
Kdx = 1.4;

Kpy = 0.8;
Kdy = 1.4;

Kpz = 2.0;
Kdz = 1.8;


%% =========================
%  10. 姿态内环增益
% ==========================

Kp_phi = 0.4;
Kd_phi = 0.15;

Kp_theta = 0.4;
Kd_theta = 0.15;

Kp_psi = 0.3;
Kd_psi = 0.12;


%% =========================
%  11. 初始化真实状态
% ==========================

% 位置
x = zeros(size(t));
y = zeros(size(t));
z = zeros(size(t));

% 速度
vx = zeros(size(t));
vy = zeros(size(t));
vz = zeros(size(t));

% 欧拉角
phi = zeros(size(t));
theta = zeros(size(t));
psi = zeros(size(t));

% 角速度
p = zeros(size(t));
q = zeros(size(t));
r = zeros(size(t));


%% =========================
%  12. 初始化控制量
% ==========================

ax_c = zeros(size(t));
ay_c = zeros(size(t));
az_c = zeros(size(t));

phi_d = zeros(size(t));
theta_d = zeros(size(t));
psi_d = zeros(size(t));

T_total = zeros(size(t));

tau_x = zeros(size(t));
tau_y = zeros(size(t));
tau_z = zeros(size(t));


%% =========================
%  13. 最大姿态角
% ==========================

angle_max = deg2rad(20);


%% =========================================================
%  14. 飞行控制 + 动力学主循环
% ==========================================================

for k = 1:N-1

    %% -------------------------
    % A. 位置误差
    % --------------------------

    ex = xd(k) - x(k);

    ey = yd(k) - y(k);

    ez = zd - z(k);


    %% -------------------------
    % B. 速度误差
    % --------------------------

    evx = vxd(k) - vx(k);

    evy = vyd(k) - vy(k);

    evz = vzd - vz(k);


    %% -------------------------
    % C. 位置外环
    %
    % 前馈 + 反馈
    %
    % a_c =
    % a_d + Kp*e + Kd*ev
    % --------------------------

    ax_c(k) = ...
          ax_d(k) ...
        + Kpx*ex ...
        + Kdx*evx;


    ay_c(k) = ...
          ay_d(k) ...
        + Kpy*ey ...
        + Kdy*evy;


    az_c(k) = ...
          az_d ...
        + Kpz*ez ...
        + Kdz*evz;


    %% -------------------------
    % D. 期望推力矢量长度
    % --------------------------

    A_c = sqrt( ...
          ax_c(k)^2 ...
        + ay_c(k)^2 ...
        + (g + az_c(k))^2 );


    %% -------------------------
    % E. 总推力
    % --------------------------

    T_total(k) = m*A_c;


    %% -------------------------
    % F. 推力矢量 -> 期望姿态
    %
    % 仍然假设 psi_d = 0
    % --------------------------

    theta_d(k) = ...
        atan2( ...
        ax_c(k), ...
        g + az_c(k));


    phi_d(k) = ...
        -asin( ...
        ay_c(k)/A_c);


    psi_d(k) = 0;


    %% -------------------------
    % G. 姿态角限制
    % --------------------------

    theta_d(k) = ...
        max( ...
        min(theta_d(k),angle_max), ...
        -angle_max);


    phi_d(k) = ...
        max( ...
        min(phi_d(k),angle_max), ...
        -angle_max);


    %% -------------------------
    % H. 姿态误差
    % --------------------------

    e_phi = ...
        phi_d(k) - phi(k);

    e_theta = ...
        theta_d(k) - theta(k);

    e_psi = ...
        psi_d(k) - psi(k);


    %% -------------------------
    % I. 姿态PD控制器
    % --------------------------

    tau_x(k) = ...
          Kp_phi*e_phi ...
        - Kd_phi*p(k);


    tau_y(k) = ...
          Kp_theta*e_theta ...
        - Kd_theta*q(k);


    tau_z(k) = ...
          Kp_psi*e_psi ...
        - Kd_psi*r(k);


    %% =====================================================
    %  Week 6 非线性动力学
    %% =====================================================


    %% -------------------------
    % J. 角速度
    % --------------------------

    omega = [
        p(k);
        q(k);
        r(k)
    ];


    %% -------------------------
    % K. 力矩
    % --------------------------

    tau = [
        tau_x(k);
        tau_y(k);
        tau_z(k)
    ];


    %% -------------------------
    % L. 旋转动力学
    % --------------------------

    omega_dot = ...
        J \ ...
        (tau ...
        - cross(omega,J*omega));


    %% -------------------------
    % M. 更新角速度
    % --------------------------

    p(k+1) = ...
        p(k) ...
        + omega_dot(1)*dt;


    q(k+1) = ...
        q(k) ...
        + omega_dot(2)*dt;


    r(k+1) = ...
        r(k) ...
        + omega_dot(3)*dt;


    %% -------------------------
    % N. Euler运动学矩阵
    % --------------------------

    phi_k = phi(k);
    theta_k = theta(k);


    E = [
        1, ...
        sin(phi_k)*tan(theta_k), ...
        cos(phi_k)*tan(theta_k);

        0, ...
        cos(phi_k), ...
        -sin(phi_k);

        0, ...
        sin(phi_k)/cos(theta_k), ...
        cos(phi_k)/cos(theta_k)
    ];


    %% -------------------------
    % O. Euler角速度
    % --------------------------

    euler_dot = E*omega;


    %% -------------------------
    % P. 更新Euler角
    % --------------------------

    phi(k+1) = ...
        phi(k) ...
        + euler_dot(1)*dt;


    theta(k+1) = ...
        theta(k) ...
        + euler_dot(2)*dt;


    psi(k+1) = ...
        psi(k) ...
        + euler_dot(3)*dt;


    %% -------------------------
    % Q. 旋转矩阵
    % --------------------------

    Rx = [
        1  0  0;

        0  cos(phi(k))  -sin(phi(k));

        0  sin(phi(k))   cos(phi(k))
    ];


    Ry = [
         cos(theta(k))  0  sin(theta(k));

         0              1  0;

        -sin(theta(k))  0  cos(theta(k))
    ];


    Rz = [
        cos(psi(k))  -sin(psi(k))  0;

        sin(psi(k))   cos(psi(k))  0;

        0             0            1
    ];


    Rmat = Rz*Ry*Rx;


    %% -------------------------
    % R. Body系推力
    % --------------------------

    F_body = [
        0;
        0;
        T_total(k)
    ];


    %% -------------------------
    % S. 转到惯性系
    % --------------------------

    F_inertial = ...
        Rmat*F_body;


    %% -------------------------
    % T. 平移动力学
    % --------------------------

    acceleration = ...
        F_inertial/m ...
        + [0;0;-g];


    %% -------------------------
    % U. 更新速度
    % --------------------------

    vx(k+1) = ...
        vx(k) ...
        + acceleration(1)*dt;


    vy(k+1) = ...
        vy(k) ...
        + acceleration(2)*dt;


    vz(k+1) = ...
        vz(k) ...
        + acceleration(3)*dt;


    %% -------------------------
    % V. 更新位置
    % --------------------------

    x(k+1) = ...
        x(k) ...
        + vx(k)*dt;


    y(k+1) = ...
        y(k) ...
        + vy(k)*dt;


    z(k+1) = ...
        z(k) ...
        + vz(k)*dt;

end


%% =========================
%  15. 补最后一个采样点
% ==========================

ax_c(end) = ax_c(end-1);
ay_c(end) = ay_c(end-1);
az_c(end) = az_c(end-1);

phi_d(end) = phi_d(end-1);
theta_d(end) = theta_d(end-1);
psi_d(end) = psi_d(end-1);

T_total(end) = T_total(end-1);

tau_x(end) = tau_x(end-1);
tau_y(end) = tau_y(end-1);
tau_z(end) = tau_z(end-1);


%% =========================================================
%  16. XY轨迹
% ==========================================================

figure;

plot(xd,yd,'--', ...
    'LineWidth',1.5);

hold on;

plot(x,y, ...
    'LineWidth',1.8);

plot(waypoints(:,1), ...
     waypoints(:,2), ...
     'o', ...
     'MarkerSize',8);

xlabel('x (m)');
ylabel('y (m)');

legend( ...
    'Desired trajectory', ...
    'Actual trajectory', ...
    'Waypoints');

title('Smooth Waypoint Tracking');

grid on;
axis equal;


%% =========================================================
%  17. x位置
% ==========================================================

figure;

plot(t,xd,'--', ...
    'LineWidth',1.5);

hold on;

plot(t,x, ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('x (m)');

legend('x_d','x');

title('X Position Tracking');

grid on;


%% =========================================================
%  18. y位置
% ==========================================================

figure;

plot(t,yd,'--', ...
    'LineWidth',1.5);

hold on;

plot(t,y, ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('y (m)');

legend('y_d','y');

title('Y Position Tracking');

grid on;


%% =========================================================
%  19. 速度
% ==========================================================

figure;

plot(t,vxd,'--', ...
    'LineWidth',1.5);

hold on;

plot(t,vx, ...
    'LineWidth',1.5);

plot(t,vyd,'--', ...
    'LineWidth',1.5);

plot(t,vy, ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Velocity (m/s)');

legend( ...
    'v_{x,d}', ...
    'v_x', ...
    'v_{y,d}', ...
    'v_y');

title('Velocity Tracking');

grid on;


%% =========================================================
%  20. 期望加速度
% ==========================================================

figure;

plot(t,ax_d, ...
    'LineWidth',1.5);

hold on;

plot(t,ay_d, ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

legend('a_{x,d}','a_{y,d}');

title('Desired Acceleration');

grid on;


%% =========================================================
%  21. Pitch
% ==========================================================

figure;

plot(t,rad2deg(theta), ...
    'LineWidth',1.5);

hold on;

plot(t,rad2deg(theta_d),'--', ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Pitch Angle (deg)');

legend('\theta','\theta_d');

title('Pitch Tracking');

grid on;


%% =========================================================
%  22. Roll
% ==========================================================

figure;

plot(t,rad2deg(phi), ...
    'LineWidth',1.5);

hold on;

plot(t,rad2deg(phi_d),'--', ...
    'LineWidth',1.5);

xlabel('Time (s)');
ylabel('Roll Angle (deg)');

legend('\phi','\phi_d');

title('Roll Tracking');

grid on;


%% =========================================================
%  23. 3D轨迹
% ==========================================================

figure;

plot3( ...
    xd, ...
    yd, ...
    zd*ones(size(t)), ...
    '--', ...
    'LineWidth',1.5);

hold on;

plot3( ...
    x,y,z, ...
    'LineWidth',1.8);

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

legend( ...
    'Desired trajectory', ...
    'Actual trajectory');

title('3D Smooth Waypoint Tracking');

grid on;
axis equal;
view(3);
