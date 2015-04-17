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
persistent X_prev t_prev_sen P Q R

if isempty(X_prev)
    X_prev = zeros(15,1);
    t_prev_sen = 0;
    P = eye(15);
    Q = eye(12);%bsxfun(@times,eye(12),[ones(1,3)*10 ones(1,3) ones(1,3) ones(1,3)]);
    R = eye(9);
    %Q = diag([ones(1,3)*0.5 ones(1,3)*0.1 ones(1,3)*1 ones(1,3)*2]);
    %R = diag([ones(1,6)*0.0001 ones(1,3)*0.001]);
end

if ~sensor.is_ready
    X = X_prev;
    X = [X(1:3);X(7:9);X(4:6);X(10:end)];
    Z = [];
    return
end
X = X_prev;
Z = [];
   
%% propagate
dt = sensor.t-t_prev_sen;

phi = X(4);
theta = X(5);
psi = X(6);
phi_dot = sensor.omg(1);
theta_dot = sensor.omg(2);
psi_dot = sensor.omg(3);

v_x = X_prev(7);
v_y = X_prev(8);
v_z = X_prev(9);
a_x = sensor.acc(1);
a_y = sensor.acc(2);
a_z = sensor.acc(3);

%{
F = eye(15) + dt * params.A2(phi,theta,psi,...
                            a_x,a_y,a_z,...
                            X_prev(13), X_prev(14), X_prev(15),...
                            X_prev(10), X_prev(12),...
                            0,0,0,...
                            0,0,...
                            phi_dot,psi_dot);
%}
%{d
F = eye(15) + dt * params.A2(phi,theta,psi,...
                            a_x,a_y,a_z,...
                            0,0,0,...
                            0,0,...
                            0,0,0,...
                            0,0,...
                            phi_dot,psi_dot);
%}
%{
bt = params.f2(phi,theta,psi,...
    a_x,a_y,a_z,...
    X_prev(13),X_prev(14),X_prev(15),...
    X_prev(10),X_prev(11),X_prev(12),...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    v_x,v_y,v_z,...
    phi_dot,theta_dot,psi_dot)*dt;
%}
%{d
bt = params.f2(phi,theta,psi,...
    a_x,a_y,a_z,...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    0,0,0,...
    v_x,v_y,v_z,...
    phi_dot,theta_dot,psi_dot)*dt;
%}
V = params.U2(phi,theta,psi)*dt;

X = X+bt;
P = F*P*F'+ V*Q*V';

%% update
if ~isempty(sensor.id)
    C = [eye(9) zeros(9,6)];

    [p,q] = estimate_pose(sensor);
    [p_dot,~] = estimate_vel(sensor);
    Z = [p;q;p_dot];

    K = P*C'/(C*P*C'+R);
    X = X+K*(Z-C*X_prev);

    P = (eye(15)-K*C)*P;
    
    Z = [Z(1:3);Z(7:9);Z(4:6)];
end

%% store new value
X_prev = X;
t_prev_sen = sensor.t;

%% returned values need to be of certain format
X = [X(1:3);X(7:9);X(4:6);X(10:end)];
end
