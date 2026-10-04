clc;
clear;

%% =========================
% Week 6C - 3D Translation + Rotation
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
t_end = 5;
t = 0:dt:t_end;

%% 3. 位置
x = zeros(size(t));
y = zeros(size(t));
z = zeros(size(t));

%% 4. 线速度
vx = zeros(size(t));
vy = zeros(size(t));
vz = zeros(size(t));

%% 5. 线加速度
ax = zeros(size(t));
ay = zeros(size(t));
az = zeros(size(t));

%% 6. 欧拉角
phi   = zeros(size(t));   % roll
theta = zeros(size(t));   % pitch
psi   = zeros(size(t));   % yaw

%% 7. 机体系角速度
p = zeros(size(t));
q = zeros(size(t));
r = zeros(size(t));

%% 8. 初始状态
phi(1)   = 0;
theta(1) = 0;
psi(1)   = 0;

p(1) = 0;
q(1) = 0;
r(1) = 0;

x(1) = 0;
y(1) = 0;
z(1) = 0;

vx(1) = 0;
vy(1) = 0;
vz(1) = 0;

%% 9. 总推力
T = m * g;

%% 10. 仿真
for k = 1:length(t)-1

    %% -------------------------
    % A. 输入力矩
    % -------------------------

    
    tau_x = 0;
    tau_y = 0;
    tau_z = 0;
    
    

    tau = [tau_x;
           tau_y;
           tau_z];


    %% -------------------------
    % B. 3D 转动动力学
    % -------------------------

    omega = [p(k);
             q(k);
             r(k)];

    omega_dot = J \ (tau - cross(omega, J*omega));

    % 角速度积分
    p(k+1) = p(k) + omega_dot(1)*dt;
    q(k+1) = q(k) + omega_dot(2)*dt;
    r(k+1) = r(k) + omega_dot(3)*dt;


    %% -------------------------
    % C. p,q,r -> 欧拉角变化率
    % -------------------------

    E = [1, sin(phi(k))*tan(theta(k)), ...
            cos(phi(k))*tan(theta(k));

         0, cos(phi(k)), ...
           -sin(phi(k));

         0, sin(phi(k))/cos(theta(k)), ...
            cos(phi(k))/cos(theta(k))];

    euler_dot = E * omega;

    % 欧拉角积分
    phi(k+1) = phi(k) ...
             + euler_dot(1)*dt;

    theta(k+1) = theta(k) ...
               + euler_dot(2)*dt;

    psi(k+1) = psi(k) ...
             + euler_dot(3)*dt;


    %% -------------------------
    % D. 当前姿态对应的旋转矩阵
    % -------------------------

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


    %% -------------------------
    % E. 机体系推力
    % -------------------------

    F_body = [0;
              0;
              T];

    %% -------------------------
    % F. 推力转换到惯性系
    % -------------------------

    F_inertial = R * F_body;


    %% -------------------------
    % G. 3D 平动动力学
    % -------------------------

    a = F_inertial/m + [0;
                        0;
                       -g];

    ax(k) = a(1);
    ay(k) = a(2);
    az(k) = a(3);


    %% -------------------------
    % H. 加速度 -> 速度
    % -------------------------

    vx(k+1) = vx(k) + ax(k)*dt;
    vy(k+1) = vy(k) + ay(k)*dt;
    vz(k+1) = vz(k) + az(k)*dt;


    %% -------------------------
    % I. 速度 -> 位置
    % -------------------------

    x(k+1) = x(k) + vx(k)*dt;
    y(k+1) = y(k) + vy(k)*dt;
    z(k+1) = z(k) + vz(k)*dt;

end

%% 11. 补最后一个加速度点
ax(end) = ax(end-1);
ay(end) = ay(end-1);
az(end) = az(end-1);


%% 12. 欧拉角
figure;

plot(t, rad2deg(phi), 'LineWidth', 1.5);
hold on;
plot(t, rad2deg(theta), 'LineWidth', 1.5);
plot(t, rad2deg(psi), 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Angle (deg)');

legend('\phi', '\theta', '\psi');

title('Euler Angles');


%% 13. 机体系角速度
figure;

plot(t, p, 'LineWidth', 1.5);
hold on;
plot(t, q, 'LineWidth', 1.5);
plot(t, r, 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');

legend('p', 'q', 'r');

title('Body Angular Velocity');


%% 14. 位置
figure;

plot(t, x, 'LineWidth', 1.5);
hold on;
plot(t, y, 'LineWidth', 1.5);
plot(t, z, 'LineWidth', 1.5);

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

legend('x', 'y', 'z');

title('3D Position');


%% 15. 三维轨迹
figure;

plot3(x, y, z, 'LineWidth', 1.5);

grid on;
axis equal;

xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');

title('3D Flight Trajectory');

view(3);