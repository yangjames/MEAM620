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
%set(gca,'xlim',qd{qn}.pos(1)+[-1 1],'ylim',qd{qn}.pos(2)+[-1 1],'zlim',qd{qn}.pos(3)+[-1 1])
if isempty(err_c)
    err_c = [0;0;0];
end

err = qd{qn}.pos_des - qd{qn}.pos;
err_d = qd{qn}.vel_des - qd{qn}.vel;
err_c = err_c + err;

int_thresh = 100;
err_c(err_c>int_thresh) = int_thresh;

% Desired roll, pitch and yaw
a = qd{qn}.acc_des+[0 0 params.grav]' + params.Kp_o*(err) + params.Kd_o*(err_d);
if a(3) < 0.5
    a(3) = 0.5;
end
if norm(a) > 2*params.grav
    alpha = (-2*a(3)*params.grav+sqrt(4*a(3)^2*params.grav^2-4*norm(a)^2*(params.grav^2-(2*params.grav)^2)))/(2*norm(a)^2);
    a = alpha*a;
end
theta_des = atan2(a(1),a(3));
phi_des = -atan2(a(2),a(3));
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
%actual_angles = [actual_angles qd{qn}.euler];
%desired_angles = [desired_angles euler_des];

% Thrust
F    = err(3)*params.Kp_t + err_d(3)*params.Kd_t + err_d(3)*params.Ki_t...
    + params.mass*(params.grav/(cos(phi_des)*cos(theta_des))+qd{qn}.acc_des(3));

% Moment
err_r = euler_des - qd{qn}.euler;
err_r_d = err_r*10-qd{qn}.omega;
M = params.Kp_m*err_r + params.Kd_m*err_r_d;
C = cross(qd{qn}.omega,params.I*qd{qn}.omega);
M= M-C;

% You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
