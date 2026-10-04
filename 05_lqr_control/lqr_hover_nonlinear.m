%% LQR Hover Control on a Nonlinear Quadrotor
clear;
clc;
close all;

%% =========================================================
% 1. Quadrotor parameters
% ==========================================================

m = 0.65;          % mass (kg)
g = 9.81;          % gravity (m/s^2)

Jx = 0.02;
Jy = 0.02;
Jz = 0.04;

J = diag([Jx Jy Jz]);


%% =========================================================
% 2. Linearized model around hover
%
% State:
% X = [x y z vx vy vz phi theta psi p q r]'
%
% Input correction:
% deltaU = [deltaT tau_x tau_y tau_z]'
% ==========================================================

A = zeros(12,12);
B = zeros(12,4);

% Position -> velocity
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Attitude -> horizontal acceleration
A(4,8) = g;       % vx_dot = g*theta
A(5,7) = -g;      % vy_dot = -g*phi

% Angular velocity -> Euler angle rate
A(7,10) = 1;      % phi_dot   = p
A(8,11) = 1;      % theta_dot = q
A(9,12) = 1;      % psi_dot   = r

% Control input
B(6,1)  = 1/m;    % vz_dot = deltaT/m

B(10,2) = 1/Jx;   % p_dot = tau_x/Jx
B(11,3) = 1/Jy;   % q_dot = tau_y/Jy
B(12,4) = 1/Jz;   % r_dot = tau_z/Jz


%% =========================================================
% 3. Check controllability
% ==========================================================

Co = ctrb(A,B);

rCo = rank(Co);

disp('Rank of controllability matrix:');
disp(rCo);


%% =========================================================
% 4. LQR design
% ==========================================================

Q = diag([ ...
    100 100 20 ...      % x y z
    2   2   4  ...      % vx vy vz
    200 200 10 ...      % phi theta psi
    1   1   1]);        % p q r

R = diag([ ...
    1 ...                % deltaT
    20 ...               % tau_x
    20 ...               % tau_y
    5]);                 % tau_z


[K,S,e] = lqr(A,B,Q,R);

disp('LQR gain K:');
disp(K);

disp('Closed-loop poles:');
disp(e);


%% =========================================================
% 5. Simulation settings
% ==========================================================

dt = 0.01;
tf = 8;

t = 0:dt:tf;
N = length(t);


%% =========================================================
% 6. Hover equilibrium
% ==========================================================

% Desired hover state
X0 = zeros(12,1);

% Hover control input
u0 = [m*g;
      0;
      0;
      0];


%% =========================================================
% 7. Initial condition
% ==========================================================

X = zeros(12,N);

X(:,1) = [ ...
     1;       % x = 1 m
    -1;       % y = -1 m
     0.5;     % z = 0.5 m

     0;       % vx
     0;       % vy
     0;       % vz

     0;       % phi
     0;       % theta
     0;       % psi

     0;       % p
     0;       % q
     0];      % r


% Store actual control input
U = zeros(4,N);


%% =========================================================
% 8. Closed-loop nonlinear simulation
% ==========================================================

for k = 1:N-1

    % -------------------------------------------------------
    % Step 1: State error
    % -------------------------------------------------------

    deltaX = X(:,k) - X0;


    % -------------------------------------------------------
    % Step 2: LQR control correction
    %
    % deltaU = -K * deltaX
    % -------------------------------------------------------

    deltaU = -K * deltaX;


    % -------------------------------------------------------
    % Step 3: Actual control input
    %
    % u = u0 + deltaU
    % -------------------------------------------------------

    u = u0 + deltaU;


    % Save control input
    U(:,k) = u;


    % -------------------------------------------------------
    % Step 4: Nonlinear quadrotor plant
    % -------------------------------------------------------

    X_dot = quadrotorNonlinearDynamics( ...
        X(:,k), u, m, g, J);


    % -------------------------------------------------------
    % Step 5: Euler integration
    %
    % X(k+1) = X(k) + X_dot*dt
    % -------------------------------------------------------

    X(:,k+1) = X(:,k) + X_dot*dt;

end


% Last control value
U(:,end) = U(:,end-1);


%% =========================================================
% 9. Position response
% ==========================================================

figure;

plot(t,X(1,:),'LineWidth',1.5);
hold on;

plot(t,X(2,:),'LineWidth',1.5);
plot(t,X(3,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

legend('x','y','z');

title('Nonlinear Quadrotor Position Response');


%% =========================================================
% 10. Velocity response
% ==========================================================

figure;

plot(t,X(4,:),'LineWidth',1.5);
hold on;

plot(t,X(5,:),'LineWidth',1.5);
plot(t,X(6,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Velocity (m/s)');

legend('v_x','v_y','v_z');

title('Velocity Response');


%% =========================================================
% 11. Attitude response
% ==========================================================

figure;

plot(t,rad2deg(X(7,:)),'LineWidth',1.5);
hold on;

plot(t,rad2deg(X(8,:)),'LineWidth',1.5);
plot(t,rad2deg(X(9,:)),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Angle (deg)');

legend('\phi','\theta','\psi');

title('Attitude Response');


%% =========================================================
% 12. Angular velocity response
% ==========================================================

figure;

plot(t,X(10,:),'LineWidth',1.5);
hold on;

plot(t,X(11,:),'LineWidth',1.5);
plot(t,X(12,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Angular Rate (rad/s)');

legend('p','q','r');

title('Angular Velocity Response');


%% =========================================================
% 13. Control input
% ==========================================================

figure;

plot(t,U(1,:),'LineWidth',1.5);
hold on;

plot(t,U(2,:),'LineWidth',1.5);
plot(t,U(3,:),'LineWidth',1.5);
plot(t,U(4,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Control Input');

legend('T','\tau_x','\tau_y','\tau_z');

title('Control Inputs');


%% =========================================================
% 14. XY trajectory
% ==========================================================

figure;

plot(X(1,:),X(2,:),'LineWidth',1.5);

hold on;

plot(X(1,1),X(2,1),'o','MarkerSize',8);
plot(X0(1),X0(2),'x','MarkerSize',10);

grid on;
axis equal;

xlabel('x (m)');
ylabel('y (m)');

legend('Trajectory','Initial Position','Target');

title('XY Trajectory');


%% =========================================================
% Nonlinear quadrotor dynamics
% ==========================================================

function X_dot = quadrotorNonlinearDynamics(X,u,m,g,J)

    %% -------------------------------------------------------
    % 1. Extract states
    % --------------------------------------------------------

    x     = X(1);
    y     = X(2);
    z     = X(3);

    vx    = X(4);
    vy    = X(5);
    vz    = X(6);

    phi   = X(7);
    theta = X(8);
    psi   = X(9);

    p     = X(10);
    q     = X(11);
    r     = X(12);


    %% -------------------------------------------------------
    % 2. Extract control inputs
    % --------------------------------------------------------

    T     = u(1);

    tau_x = u(2);
    tau_y = u(3);
    tau_z = u(4);


    %% -------------------------------------------------------
    % 3. Rotation matrix
    %
    % Body frame -> Inertial frame
    %
    % R = Rz * Ry * Rx
    % --------------------------------------------------------

    Rx = [ ...
        1, 0,        0;
        0, cos(phi), -sin(phi);
        0, sin(phi),  cos(phi)];


    Ry = [ ...
         cos(theta), 0, sin(theta);
         0,          1, 0;
        -sin(theta), 0, cos(theta)];


    Rz = [ ...
        cos(psi), -sin(psi), 0;
        sin(psi),  cos(psi), 0;
        0,         0,        1];


    R = Rz * Ry * Rx;


    %% -------------------------------------------------------
    % 4. Thrust force
    % --------------------------------------------------------

    F_body = [ ...
        0;
        0;
        T];


    F_inertial = R * F_body;


    %% -------------------------------------------------------
    % 5. Translational dynamics
    %
    % a = F/m + gravity
    % --------------------------------------------------------

    acceleration = ...
        F_inertial/m ...
        + [0;
           0;
          -g];


    ax = acceleration(1);
    ay = acceleration(2);
    az = acceleration(3);


    %% -------------------------------------------------------
    % 6. Rotational dynamics
    %
    % J*omega_dot + omega x (J*omega) = tau
    % --------------------------------------------------------

    omega = [ ...
        p;
        q;
        r];


    tau = [ ...
        tau_x;
        tau_y;
        tau_z];


    omega_dot = ...
        J \ (tau - cross(omega,J*omega));


    p_dot = omega_dot(1);
    q_dot = omega_dot(2);
    r_dot = omega_dot(3);


    %% -------------------------------------------------------
    % 7. Euler angle kinematics
    %
    % Euler_dot = E * omega
    % --------------------------------------------------------

    E = [ ...
        1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
        0, cos(phi),            -sin(phi);
        0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];


    euler_dot = E * omega;


    phi_dot   = euler_dot(1);
    theta_dot = euler_dot(2);
    psi_dot   = euler_dot(3);


    %% -------------------------------------------------------
    % 8. Assemble state derivative
    % --------------------------------------------------------

    X_dot = [ ...
        vx;
        vy;
        vz;

        ax;
        ay;
        az;

        phi_dot;
        theta_dot;
        psi_dot;

        p_dot;
        q_dot;
        r_dot];

end
