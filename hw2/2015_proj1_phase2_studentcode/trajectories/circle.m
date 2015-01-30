function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
r = 5;
w = 1;

if w*t <= 2*pi
    pos = [r*cos(w*t); r*sin(w*t); (w*t)*2.5/(2*pi)];
    vel = [-r*w*sin(w*t); r*w*cos(w*t); w*2.5/(2*pi)];
    acc = [-r*w^2*cos(w*t); -r*w^2*sin(w*t); 0];
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
