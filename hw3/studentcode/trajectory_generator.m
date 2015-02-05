function [ desired_state ] = trajectory_generator(t, qn, map, path)
persistent path0 c_time dt_stamps t_idx t_total pivot_idx
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;

if isempty(path0)
    path0 = path{1};
    max_snap = 17;
    
    % find indices where path switches directions
    pivot_idx = [find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;size(path0,1)];
    
    % calculate distance between pivot points
    distances = sqrt(sum((path0([1; pivot_idx],:)-path0([pivot_idx;size(path0,1)],:)).^2,2));
    
    % gather time stamps between pivot points
    dt_stamps = (24*distances/max_snap).^(1/4);
    
    c_time = dt_stamps(1);
    t_idx = 1;
    t_total = sum(dt_stamps);
end

if t < t_total-dt_stamps(end)
    if t > c_time
        t_idx = t_idx+1;
        c_time = c_time + dt_stamps(t_idx);
    end
    dt = t-sum(dt_stamps(1:t_idx-1));
    X_0 = [path0(pivot_idx(t_idx),:);...
        0 0 0;...
        0 0 0;...
        path0(pivot_idx(t_idx+1),:);...
        0 0 0;...
        0 0 0];
    a = get_interp_weights(X_0,dt_stamps(t_idx));
    X = [1 dt dt^2 dt^3 dt^4 dt^5;...
        0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;...
        0 0 2 6*dt 12*dt^2 20*dt^3]*a;
    pos = X(1,:)';
    vel = X(2,:)';
    acc = X(3,:)';
else
    pos = path0(end,:)';
    vel = [0 0 0]';
    acc = [0 0 0]';
end

yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;