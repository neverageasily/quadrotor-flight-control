m = 0.65;
g = 9.81;
Fd=1;

dt = 0.01;
t_end = 30;
t = 0:dt:t_end;

z_d=2;
Kp=2;
Ki=0.5;
Kd=4;
integral_e=0;

z = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));

e = zeros(size(t));
T = zeros(size(t));

z(1) = 0;
v(1) = 0;
for k = 1:length(t)-1
    e(k)=z_d-z(k);
    integral_e=integral_e+e(k)*dt;
    T(k)=m*g+Kp*e(k)+Ki*integral_e-Kd*v(k);
    a(k)=(T(k)-m*g-Fd)/m;
    
    v(k+1)=v(k)+a(k)*dt;
    z(k+1)=z(k)+v(k)*dt;
end
e(end)=e(end-1);
T(end)=T(end-1);
a(end)=a(end-1);

figure;
plot(t,z);
grid on;
xlabel("Time (s)");
ylabel("Height z (m)");
title("PID Control - Altitude");

figure;
plot(t,e);
grid on;
xlabel("Time (s)");
ylabel("Error e (m)");
title("Altitude Error");

figure;
plot(t,T);
grid on;
xlabel("Time (s)");
ylabel("Thrust T (N)");
title("Thrust");