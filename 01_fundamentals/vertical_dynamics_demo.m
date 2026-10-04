m = 0.65;
g = 9.81;

dt = 0.001;
t_end = 5;
t=0:dt:t_end;

z=zeros(size(t));
v=zeros(size(t));
a=zeros(size(t));

z(1)=0;
v(1)=0;

T=1.1*m*g;

for k=1:length(t)-1;
    a(k)=(T-m*g)/m;
    v(k+1)=v(k)+a(k)*dt;
    z(k+1)=z(k)+v(k)*dt;
end
a(end)=a(end-1);

figure;
plot(t,z);
grid on;
xlabel("Time(s)");
ylabel("Height(m)");
title("Altitude");

figure;
plot(t,v);
grid on;
xlabel("Time(s)");
ylabel("Velocity(m/s)");
title("Velocity");

figure;
plot(t,a);
grid on;
xlabel("Time(s)");
ylabel("acceleration(m/s^2)");
title("acceleration");

disp(z(end));
disp(v(end));
disp(a(end));