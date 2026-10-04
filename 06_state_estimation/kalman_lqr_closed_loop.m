%% Kalman Filter + LQR + Nonlinear Quadrotor
clear;
clc;
close all;

rng(1);


%% =========================================================
% 1. Quadrotor parameters
% ==========================================================

m = 0.65;
g = 9.81;

Jx = 0.02;
Jy = 0.02;
Jz = 0.04;

J = diag([Jx Jy Jz]);


%% =========================================================
% 2. Linearized model around hover
%
% X =
% [x y z vx vy vz phi theta psi p q r]'
%
% deltaU =
% [deltaT tau_x tau_y tau_z]'
% ==========================================================

A = zeros(12,12);
B = zeros(12,4);

% Position
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

% Horizontal acceleration
A(4,8) = g;
A(5,7) = -g;

% Euler angle kinematics
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

% Control inputs
B(6,1)  = 1/m;

B(10,2) = 1/Jx;
B(11,3) = 1/Jy;
B(12,4) = 1/Jz;


%% =========================================================
% 3. LQR design
% ==========================================================

Q_lqr = diag([ ...
    100 100 20 ...
    2   2   4  ...
    200 200 10 ...
    1   1   1]);

R_lqr = diag([1 20 20 5]);

K_lqr = lqr(A,B,Q_lqr,R_lqr);


%% =========================================================
% 4. Simulation settings
% ==========================================================

dt = 0.01;
tf = 8;

t = 0:dt:tf;
N = length(t);


%% =========================================================
% 5. Discrete model for Kalman Filter
% ==========================================================

Ad = eye(12) + A*dt;
Bd = B*dt;


%% =========================================================
% 6. Measurement model
%
% We measure:
%
% x y z phi theta psi
%
% We do NOT directly measure:
%
% vx vy vz p q r
% ==========================================================

H = zeros(6,12);

H(1,1) = 1;       % x
H(2,2) = 1;       % y
H(3,3) = 1;       % z

H(4,7) = 1;       % phi
H(5,8) = 1;       % theta
H(6,9) = 1;       % psi


%% =========================================================
% 7. KF noise covariance
% ==========================================================

% Process noise covariance
%
% The KF assumes the linear model is not perfect.

Q_kf = 1e-4 * eye(12);


% Measurement noise standard deviation

sigma_pos   = 0.1;          % position: 0.1 m
sigma_angle = deg2rad(1);    % attitude: 1 degree


% Measurement noise covariance

R_kf = diag([ ...
    sigma_pos^2 ...
    sigma_pos^2 ...
    sigma_pos^2 ...
    sigma_angle^2 ...
    sigma_angle^2 ...
    sigma_angle^2]);


%% =========================================================
% 8. Check observability
% ==========================================================

Ob = obsv(Ad,H);

disp('Rank of observability matrix:');
disp(rank(Ob));


%% =========================================================
% 9. Hover equilibrium
% ==========================================================

X0 = zeros(12,1);

u0 = [m*g;
      0;
      0;
      0];


%% =========================================================
% 10. True initial state
% ==========================================================

X_true = zeros(12,N);

X_true(:,1) = [ ...
     1;
    -1;
     0.5;

     0;
     0;
     0;

     0;
     0;
     0;

     0;
     0;
     0];


%% =========================================================
% 11. Kalman Filter initialization
% ==========================================================

X_hat = zeros(12,N);

% Initial covariance
P = eye(12);


%% =========================================================
% 12. Storage
% ==========================================================

% Measurements
Z = zeros(6,N);

% Actual control inputs
U = zeros(4,N);

% Previous control correction
deltaU_previous = zeros(4,1);


%% =========================================================
% 13. KF + LQR closed-loop simulation
% ==========================================================

for k = 1:N

    %% -------------------------------------------------------
    % A. Sensor measurement
    %% --------------------------------------------------------

    measurement_noise = [ ...
        sigma_pos   * randn;
        sigma_pos   * randn;
        sigma_pos   * randn;

        sigma_angle * randn;
        sigma_angle * randn;
        sigma_angle * randn];


    Z(:,k) = ...
        H * X_true(:,k) ...
        + measurement_noise;


    %% =====================================================
    % B. Kalman Filter - Prediction
    %% =====================================================

    if k == 1

        % Initial prior estimate
        X_hat_minus = zeros(12,1);

        P_minus = P;

    else

        % State prediction
        %
        % Xhat_k- =
        % Ad * Xhat_(k-1)+
        % + Bd * deltaU_(k-1)

        X_hat_minus = ...
            Ad * X_hat(:,k-1) ...
            + Bd * deltaU_previous;


        % Covariance prediction
        %
        % P_k- =
        % Ad * P_(k-1)+ * Ad'
        % + Q

        P_minus = ...
            Ad * P * Ad' ...
            + Q_kf;

    end


    %% =====================================================
    % C. Kalman Filter - Measurement Update
    %% =====================================================

    % Measurement residual / innovation
    %
    % y = z - H*Xhat-

    innovation = ...
        Z(:,k) ...
        - H * X_hat_minus;


    % Kalman gain
    %
    % K =
    % P- H' / (H P- H' + R)

    Kf = ...
        P_minus * H' / ...
        (H * P_minus * H' + R_kf);


    % State update
    %
    % Xhat+ =
    % Xhat- + K*innovation

    X_hat(:,k) = ...
        X_hat_minus ...
        + Kf * innovation;


    % Covariance update
    %
    % P+ =
    % (I - K H) P-

    P = ...
        (eye(12) - Kf*H) ...
        * P_minus;


    %% =====================================================
    % D. LQR control using ESTIMATED state
    %% =====================================================

    deltaX_hat = ...
        X_hat(:,k) - X0;


    % IMPORTANT:
    %
    % LQR uses X_hat,
    % NOT X_true

    deltaU = ...
        -K_lqr * deltaX_hat;


    % Actual control input

    u = ...
        u0 + deltaU;


    U(:,k) = u;


    %% =====================================================
    % E. Nonlinear true quadrotor plant
    %% =====================================================

    if k < N

        X_dot = ...
            quadrotorNonlinearDynamics( ...
            X_true(:,k), ...
            u, ...
            m, ...
            g, ...
            J);


        X_true(:,k+1) = ...
            X_true(:,k) ...
            + X_dot*dt;


        % Save control correction for next KF prediction

        deltaU_previous = deltaU;

    end

end


%% =========================================================
% 14. Position estimation: x
% ==========================================================

figure;

plot(t,X_true(1,:),'LineWidth',1.5);
hold on;

plot(t,Z(1,:),'.');

plot(t,X_hat(1,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('x (m)');

legend( ...
    'True x', ...
    'Noisy Measurement', ...
    'KF Estimate');

title('Kalman Filter - x Position');


%% =========================================================
% 15. Velocity estimation: vx
%
% Note:
% vx is NOT directly measured.
% ==========================================================

figure;

plot(t,X_true(4,:),'LineWidth',1.5);
hold on;

plot(t,X_hat(4,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('v_x (m/s)');

legend( ...
    'True v_x', ...
    'KF Estimate');

title('Kalman Filter - v_x Estimation');


%% =========================================================
% 16. Position control
% ==========================================================

figure;

plot(t,X_true(1,:),'LineWidth',1.5);
hold on;

plot(t,X_true(2,:),'LineWidth',1.5);
plot(t,X_true(3,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Position (m)');

legend('x','y','z');

title('KF + LQR: Quadrotor Position');


%% =========================================================
% 17. Attitude estimation: phi
% ==========================================================

figure;

plot(t,rad2deg(X_true(7,:)), ...
    'LineWidth',1.5);

hold on;

plot(t,rad2deg(Z(4,:)),'.');

plot(t,rad2deg(X_hat(7,:)), ...
    'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('\phi (deg)');

legend( ...
    'True \phi', ...
    'Noisy Measurement', ...
    'KF Estimate');

title('Kalman Filter - Roll Angle');


%% =========================================================
% 18. Angular rate estimation: p
%
% p is NOT directly measured.
% ==========================================================

figure;

plot(t,X_true(10,:),'LineWidth',1.5);
hold on;

plot(t,X_hat(10,:),'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('p (rad/s)');

legend( ...
    'True p', ...
    'KF Estimate');

title('Kalman Filter - p Estimation');


%% =========================================================
% 19. Control inputs
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

legend( ...
    'T', ...
    '\tau_x', ...
    '\tau_y', ...
    '\tau_z');

title('KF + LQR Control Inputs');


%% =========================================================
% Nonlinear quadrotor dynamics
% ==========================================================

function X_dot = quadrotorNonlinearDynamics(X,u,m,g,J)

    %% States

    vx = X(4);
    vy = X(5);
    vz = X(6);

    phi   = X(7);
    theta = X(8);
    psi   = X(9);

    p = X(10);
    q = X(11);
    r = X(12);


    %% Inputs

    T = u(1);

    tau_x = u(2);
    tau_y = u(3);
    tau_z = u(4);


    %% Rotation matrices

    Rx = [ ...
        1  0         0;
        0  cos(phi) -sin(phi);
        0  sin(phi)  cos(phi)];


    Ry = [ ...
         cos(theta)  0  sin(theta);
         0           1  0;
        -sin(theta)  0  cos(theta)];


    Rz = [ ...
        cos(psi) -sin(psi)  0;
        sin(psi)  cos(psi)  0;
        0         0         1];


    R = Rz * Ry * Rx;


    %% Translational dynamics

    F_body = [0;
              0;
              T];


    F_inertial = R * F_body;


    acceleration = ...
        F_inertial/m ...
        + [0;
           0;
          -g];


    ax = acceleration(1);
    ay = acceleration(2);
    az = acceleration(3);


    %% Rotational dynamics

    omega = [p;
             q;
             r];


    tau = [tau_x;
           tau_y;
           tau_z];


    omega_dot = ...
        J \ (tau - cross(omega,J*omega));


    p_dot = omega_dot(1);
    q_dot = omega_dot(2);
    r_dot = omega_dot(3);


    %% Euler angle kinematics

    E = [ ...
        1, sin(phi)*tan(theta), ...
           cos(phi)*tan(theta);

        0, cos(phi), ...
          -sin(phi);

        0, sin(phi)/cos(theta), ...
           cos(phi)/cos(theta)];


    euler_dot = E * omega;


    phi_dot   = euler_dot(1);
    theta_dot = euler_dot(2);
    psi_dot   = euler_dot(3);


    %% State derivative

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
