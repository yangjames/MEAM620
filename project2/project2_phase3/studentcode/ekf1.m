function [X, Z] = ekf1(sensor, vic, params)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; roll; pitch; yaw; other states you use]
%     we will only take the first 6 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; roll; pitch; yaw; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement 
persistent X_prev t_prev_vic t_prev_sen P Q R

if isempty(X_prev)
    X_prev = zeros(6,1);
    t_prev_vic = 0;
    t_prev_sen = 0;
    P = eye(6);
    Q = diag([1 1 1 1 1 1])*0.01;
    R = diag([1 1 1 10 10 10]);
end

X = X_prev;
Z = zeros(6,1);
if isempty(vic) && isempty(sensor)
    X = [];
    Z = [];
    return
end

%% if vicon data is available, propagate
if ~isempty(vic)
    dt = vic.t - t_prev_vic;
    
    phi = X(4);
    theta = X(5);
    phi_dot = vic.vel(4);
    psi_dot = vic.vel(6);
    
    F = eye(6) + dt * params.A1(phi,theta,0,0,phi_dot,psi_dot);
    
    bt = params.f1(phi,theta,...
        0,0,0,...
        0,0,0,...
        vic.vel(1),vic.vel(2),vic.vel(3),...
        vic.vel(4),vic.vel(5),vic.vel(6))*dt;
    V = params.U1(phi,theta)*dt;
    
    X = X+bt;
    P = F*P*F'+ V*Q*V';
    t_prev_vic = vic.t;
end

%% if April Tag info available, update
if ~isempty(sensor)
    dt = sensor.t-t_prev_sen;
    
    %% create C matrix
    C = eye(6);

    %% update
    [p,q] = estimate_pose(sensor);
    Z = [p;q];

    K = P*C'/(C*P*C'+R);
    X = X+K*(Z-C*X);

    P = (eye(6)-K*C)*P;
    
    t_prev_sen = sensor.t;
end

%% store new value
X_prev = X;
end
