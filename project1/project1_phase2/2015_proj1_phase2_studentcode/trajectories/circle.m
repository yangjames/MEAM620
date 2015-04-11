function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
r = 5;
t_final = 11;
a = get_interp_weights([0;0;0;2*pi;0;0],t_final);
X = [1 t t^2 t^3 t^4 t^5;...
        0 1 2*t 3*t^2 4*t^3 5*t^4;...
        0 0 2 6*t 12*t^2 20*t^3]*a;
theta = X(1);
theta_dot = X(2);
theta_ddot = X(3);
if t < t_final
    pos = [r*cos(theta); r*sin(theta); theta*2.5/(2*pi)];
    vel = [-r*theta_dot*sin(theta); r*theta_dot*cos(theta); theta_dot*2.5/(2*pi)];
    acc = [-r*theta_ddot*sin(theta)-r*theta_dot^2*cos(theta); r*theta_ddot*cos(theta)-r*theta_dot^2*sin(theta); theta_ddot*2.5/(2*pi)];
else
    pos = [5 0 2.5]';
    vel = [0 0 0]';
    acc = [0 0 0]';
end
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
