m = 0.65;
g = 9.81;

dt = 0.01;
t_end=5;
t=0:dt:t_end;

x=zeros(size(t));
z=zeros(size(t));
vx=zeros(size(t));
vz=zeros(size(t));
ax=zeros(size(t));
az=zeros(size(t));

x(1)=0;
z(1)=0;
vx(1)=0;
vz(1)=0;

T=m*g;

theta=zeros(size(t));
q=zeros(size(t));
alpha=zeros(size(t));
theta(1)=0;
q(1)=0;

Jy=0.02;

for k=1:length(t)-1
    if t(k)<1
        tau_y=0.01;
    elseif t(k)<2
        tau_y=-0.01'
    else
        tau_y=0;
    end
    
    alpha(k)=tau_y/Jy;
    q(k+1)=q(k)+alpha(k)*dt;
    theta(k+1)=theta(k)+q(k)*dt;
    
    ax(k)=T*sin(theta(k))/m;
    az(k)=T*cos(theta(k))/m-g;

    vx(k+1)=vx(k)+ax(k)*dt;
    vz(k+1)=vz(k)+az(k)*dt;

    x(k+1)=x(k)+vx(k)*dt;
    z(k+1)=z(k)+vz(k)*dt;
end

alpha(end)=alpha(end-1);
ax(end)=ax(end-1);
az(end)=az(end-1);

disp(ax(end));
disp(az(end));

figure;
plot(t,x);
grid on;
xlabel("Time(s)");
ylabel("x(m)");
title("Horizontal Position");

figure;
plot(t,z);
grid on;
xlabel("Time(s)");
ylabel("z(m)");
title("Vertical Position");

figure;
plot(x,z);
grid on;
xlabel("x(m)");
ylabel("z(m)");
title("2D Trajectory");

figure;
plot(t,rad2deg(theta));
grid on;
xlabel("Time (s)");
ylabel("theta (deg)");
title("Pitch Angle");

figure;
plot(t,q);
grid on;
xlabel("Time (s)");
ylabel("q (rad/s)");
title("Pitch Angular Velocity");

figure;
plot(t,ax);
grid on;
xlabel("Time (s)");
ylabel("a_x (m/s^2)");
title("Horizontal Acceleration");

figure;
plot(t,az);
grid on;
xlabel("Time (s)");
ylabel("a_z (m/s^2)");
title("Vertical Acceleration");