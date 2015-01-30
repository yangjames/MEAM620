function [F, M, trpy, drpy] = controller(qd, t, qn, params)
persistent err_c Kp_o Kd_o Ki_o Kp_t Kd_t Ki_t Kp_m Kd_m Kp_m_y Kd_m_y
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
if isempty(err_c)
    err_c = [0;0;0];
    % orientation gains
    Kp_o = 1.5; Kd_o = 1; Ki_o = 0.00015;
    
    % thrust gains
    Kp_t = 2; Kd_t = 2; Ki_t = 0.000;
    
    % moment gains
    Kp_m = 4; Kd_m = 0.5;
    
    Kp_m_y = 0.1; Kd_m_y = 0;
end
%{
%% planar
qd{qn}.pos_des(3) = 0;
qd{qn}.vel_des(3) = 0;
%}
%{
%% altitude
qd{qn}.pos_des(1) = 0;
qd{qn}.pos_des(2) = 0;
qd{qn}.vel_des(1) = 0;
qd{qn}.vel_des(2) = 0;
%}
err = qd{qn}.pos_des - qd{qn}.pos;
err_d = qd{qn}.vel_des - qd{qn}.vel;
err_c = err_c + err;

int_thresh = 1000;
err_c(err_c>int_thresh) = int_thresh;

% Desired roll, pitch and yaw
phi_des = -(err(2)*Kp_o + err_d(2)*Kd_o + err_c(2)*Ki_o);
theta_des = err(1)*Kp_o + err_d(1)*Kd_o + err_c(1)*Ki_o;
psi_des = 0;
max_angle = pi/3;
if phi_des > max_angle
    phi_des = max_angle;
elseif phi_des < -max_angle
    phi_des = -max_angle;
end
if theta_des > max_angle
    theta_des = max_angle;
elseif theta_des < -max_angle
    theta_des = -max_angle;
end

euler_des = [phi_des; theta_des; psi_des];

% Thrust
F    = err(3)*Kp_t + err_d(3)*Kd_t + err_d(3)*Ki_t + params.mass*params.grav/(cos(phi_des)*cos(theta_des));

% Moment
R = euler_to_rot(qd{qn}.euler(1),qd{qn}.euler(2),qd{qn}.euler(3));
err_r = euler_des - qd{qn}.euler;
err_r_d = -R*qd{qn}.omega;
M = Kp_m*err_r + Kd_m*err_r_d;
%M    = zeros(3,1);

% You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
