function [X, Z] = ekf2(sensor, params)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; roll; pitch; yaw; other states you use]
%     we will only take the first 9 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement 
persistent X_prev t_prev_vic t_prev_sen P Q R

if isempty(X_prev)
    X_prev = zeros(15,1);
    t_prev_vic = 0;
    t_prev_sen = 0;
    P = eye(15);
    Q = diag([1 1 1 1 1 1 1 1 1 1 1 1])*2;
    R = diag([1 1 1 10 10 10]);
end

if isempty(sensor)
    X = [];
    Z = [];
    return
end

X = X_prev;

%% if april tag data is available, run EKF
   
% propagate
dt = sensor.t-t_prev_sen;

phi = X(4);
theta = X(5);
psi = X(6);
phi_dot = sensor.omg(1);
theta_dot = sensor.omg(2);
psi_dot = sensor.omg(3);

a_x = sensor.acc(1);
a_y = sensor.acc(2);
a_z = sensor.acc(3);

F = eye(15) + dt * params.A2(phi,theta,psi,...
                            a_x,a_y,a_z,...
                            X_prev(13), X_prev(14), X_prev(15),...
                            X_prev(10), X_prev(12),...
                            0,0,0,...
                            0,0,...
                            phi_dot,psi_dot);

bt = params.f2(phi,theta,psi,...
    a_x,a_y,a_z,...
    X_prev(13),X_prev(14),X_prev(15),...
    X_prev(10),X_prev(11),X_prev(12),...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    X_prev(7),X_prev(8),X_prev(9),...
    phi_dot,theta_dot,psi_dot)*dt;
V = params.U2(phi,theta,psi)*dt;

X = X+bt;
P = F*P*F'+ V*Q*V';

% update
%% create C matrix
C = [eye(6) zeros(6,9)];

%% update
[p,q] = estimate_pose(sensor);
Z = [p;q];

K = P*C'/(C*P*C'+R);
X = X+K*(Z-C*X);

P = (eye(15)-K*C)*P;

t_prev_sen = sensor.t;

%% store new value
X_prev = X;
end
