function [F, M, trpy, drpy] = controller(qd, t, qn, params)
persistent err_c
global desired_angles actual_angles
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
if isempty(err_c)
    err_c = [0;0;0];
end

err = qd{qn}.pos_des - qd{qn}.pos;
err_d = qd{qn}.vel_des - qd{qn}.vel;
err_c = err_c + err;

int_thresh = 100;
err_c(err_c>int_thresh) = int_thresh;

% Desired roll, pitch and yaw
%{
a_dir = euler_to_rot(0,0,qd{qn}.euler(3))*(err/norm(err)+[0 0 params.grav]');

phi_des = 
theta_des = 
psi_des = 0;
%}
%{d
phi_des = -(err(2)*params.Kp_o + err_d(2)*params.Kd_o + err_c(2)*params.Ki_o);
    %+(qd{qn}.acc_des(1)*cos(qd{qn}.euler(3)) + qd{qn}.acc_des(2)*sin(qd{qn}.euler(3)))/params.grav;
theta_des = err(1)*params.Kp_o + err_d(1)*params.Kd_o + err_c(1)*params.Ki_o;
     %-(qd{qn}.acc_des(1)*sin(qd{qn}.euler(3))-qd{qn}.acc_des(2)*cos(qd{qn}.euler(3)))/params.grav;
%theta_des = -(qd{qn}.acc_des(1)*sin(qd{qn}.euler(3)) - qd{qn}.acc_des(2)*cos(qd{qn}.euler(3)))/params.grav;
%phi_des = (qd{qn}.acc_des(1)*cos(qd{qn}.euler(3)) + qd{qn}.acc_des(2)*sin(qd{qn}.euler(3)))/params.grav;
psi_des = 0;

if phi_des > params.maxangle
    phi_des = params.maxangle;
elseif phi_des < -params.maxangle
    phi_des = -params.maxangle;
end
if theta_des > params.maxangle
    theta_des = params.maxangle;
elseif theta_des < -params.maxangle
    theta_des = -params.maxangle;
end

euler_des = [phi_des; theta_des; psi_des];
actual_angles = [actual_angles qd{qn}.euler];
desired_angles = [desired_angles euler_des];
%}
% Thrust
F    = err(3)*params.Kp_t + err_d(3)*params.Kd_t + err_d(3)*params.Ki_t...
    + params.mass*(params.grav/(cos(phi_des)*cos(theta_des))+qd{qn}.acc_des(3));

% Moment
R = euler_to_rot(qd{qn}.euler(1),qd{qn}.euler(2),qd{qn}.euler(3));
err_r = euler_des - qd{qn}.euler;
err_r_d = -qd{qn}.omega;
M = params.Kp_m*err_r + params.Kd_m*err_r_d;
M(3) = params.Kp_m_y*err_r(3) + params.Kd_m_y*err_r_d(3);
C = cross(qd{qn}.omega,params.I*qd{qn}.omega);
M= M-C;
%M    = zeros(3,1);

% You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
