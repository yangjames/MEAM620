function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
duration = 3;

if t < duration
    pos = [1/(4*duration)*t;...
        sqrt(2)/duration*t;...
        sqrt(2)/duration*t];
    vel = [1/(4*duration);...
        sqrt(2)/duration;...
        sqrt(2)/duration];
    acc = [0;0;0];
elseif t < duration*2
    pos = [1/(4*duration)*(t-duration);...
        -sqrt(2)/duration*(t-duration);...
        sqrt(2)/duration*(t-duration)]...
        +[1/4; sqrt(2); sqrt(2)];
    vel = [1/(4*duration);...
        -sqrt(2)/duration;...
        sqrt(2)/duration];
    acc = [0;0;0];
elseif t < duration*3
    pos = [1/(4*duration)*(t-duration*2);...
        -sqrt(2)/duration*(t-duration*2);...
        -sqrt(2)/duration*(t-duration*2)]...
        +[2/4;0;2*sqrt(2)];
    vel = [1/(4*duration); -sqrt(2)/duration; -sqrt(2)/duration];
    acc = [0;0;0];
elseif t < duration*4
    pos = [1/(4*duration)*(t-duration*3);...
        sqrt(2)/duration*(t-duration*3);...
        -sqrt(2)/duration*(t-duration*3)]...
        +[3/4;-sqrt(2);sqrt(2)];
    vel = [1/(4*duration);...
        sqrt(2)/duration;...
        -sqrt(2)/duration];
    acc = [0;0;0];
elseif t >= duration*4
    pos = [1;0;0];
    vel = [0;0;0];
    acc = [0;0;0];
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
