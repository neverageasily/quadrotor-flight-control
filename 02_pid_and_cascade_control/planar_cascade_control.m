m=0.65;
g=9.81;
Jy=0.02;

dt=0.01;
t_end=10;
t=0:dt:t_end;

xd=-3;
Kpx=1;
Kdx=1.5;

Kptheta=0.4;
Kdtheta=0.15;

x = zeros(size(t));
vx = zeros(size(t));
ax = zeros(size(t));

theta = zeros(size(t));
q = zeros(size(t));
alpha = zeros(size(t));

ax_d = zeros(size(t));
theta_d = zeros(size(t));
tau_y = zeros(size(t));

theta_max=deg2rad(20);

zd=2;
z = zeros(size(t));
vz = zeros(size(t));
az = zeros(size(t));
ez = zeros(size(t));

Kpz = 2.0;
Kiz = 0;
Kdz = 1.5;

integral_ez = 0;

for k=1:length(t)-1
    ex=xd-x(k);
    ax_d(k)=Kpx*ex-Kdx*vx(k);

    theta_d(k)=ax_d(k)/g;

    if theta_d(k)>theta_max
        theta_d(k)=theta_max;
    elseif theta_d(k)<-theta_max
        theta_d(k)=-theta_max;
    end

    e_theta=theta_d(k)-theta(k);
    tau_y(k)=Kptheta*e_theta-Kdtheta*q(k);

    alpha(k)=tau_y(k)/Jy;
    q(k+1)=q(k)+alpha(k)*dt;
    theta(k+1)=theta(k)+q(k)*dt;

    % Compute altitude thrust before using it in the horizontal dynamics.
    ez(k) = zd-z(k);
    integral_ez=integral_ez+ez(k)*dt;
    T=(m*g+Kpz*ez(k)+Kiz*integral_ez-Kdz*vz(k))/cos(theta(k));

    ax(k)=T*sin(theta(k))/m;
    vx(k+1)=vx(k)+ax(k)*dt;
    x(k+1)=x(k)+vx(k)*dt;

    az(k) = (T * cos(theta(k)) - m*g) / m;
    vz(k+1) = vz(k) + az(k) * dt;
    z(k+1) = z(k) + vz(k) * dt;
end

ax_d(end) = ax_d(end-1);
theta_d(end) = theta_d(end-1);
tau_y(end) = tau_y(end-1);
alpha(end) = alpha(end-1);
ax(end) = ax(end-1);

figure;
plot(t, x, 'LineWidth', 1.5);
hold on;
yline(xd, '--');
grid on;
xlabel('Time (s)');
ylabel('x (m)');
title('Horizontal Position');

figure;
plot(t, theta, 'LineWidth', 1.5);
hold on;
plot(t, theta_d, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('\theta (rad)');
legend('\theta', '\theta_d');
title('Pitch Angle Tracking');

figure;
plot(t, z, 'LineWidth', 1.5);
hold on;
yline(zd, '--');
grid on;
xlabel('Time (s)');
ylabel('z (m)');
title('Altitude');

figure;
plot(x, z, 'LineWidth', 1.5);
hold on;
plot(xd, zd, 'o', 'MarkerSize', 8);
grid on;
axis equal;

xlabel('x (m)');
ylabel('z (m)');
title('2D Flight Trajectory');
legend('Trajectory', 'Target');
