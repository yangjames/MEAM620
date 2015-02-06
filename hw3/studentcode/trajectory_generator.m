function [ desired_state ] = trajectory_generator(t, qn, map, path)
persistent path0 c_time dt_stamps t_idx t_total pivot_idx C
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

%{
if isempty(path0)
    path0 = path{1};
    max_der = 17;
    n_der = 4;
    
    % find indices where path switches directions
    pivot_idx = [find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;size(path0,1)];
    
    % calculate distance between pivot points
    distances = sqrt(sum((path0([1; pivot_idx],:)-path0([pivot_idx;size(path0,1)],:)).^2,2));
    
    % gather time stamps between pivot points
    dt_stamps = (factorial(n_der)*distances/max_der).^(1/n_der);
    
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
%}
if isempty(C)
    path0 = path{1};
    max_der = 50;
    n_der = 4;

    % find indices where path switches directions
    pivot_idx = find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;

    truncated_path = [path0(1,:); path0(pivot_idx,:); path0(end,:)];

    % calculate distance between pivot points
    distances = sqrt(sum((truncated_path(1:end-1,:)-truncated_path(2:end,:)).^2,2));

    % gather time stamps between pivot points
    dt_stamps = cumsum([0;(factorial(n_der)*distances/max_der).^(1/n_der)]);
    
    A = zeros(length(distances)*4);
    X = zeros(length(distances)*4,3);

    % add position constraints
    for i = 1:length(distances)
        A((i-1)*2+1:(i-1)*2+2,(i-1)*4+1:(i-1)*4+4) = [1 dt_stamps(i) dt_stamps(i)^2 dt_stamps(i)^3;...
                                1 dt_stamps(i+1) dt_stamps(i+1)^2 dt_stamps(i+1)^3];
        X((i-1)*2+1:(i-1)*2+2,:) = [truncated_path(i,:);truncated_path(i+1,:)];
    end

    % add end point velocity constraints
    A(length(distances)*2+1,1:4) = [0 1 dt_stamps(1) dt_stamps(1)^2];
    A(length(distances)*2+2,end-3:end) = [0 1 dt_stamps(end) dt_stamps(end)^2];

    % add velocity and acceleration constraints
    for i = 1:length(distances)-1
        A(length(distances)*2+2+i,(i-1)*4+1:(i-1)*4+8)=...
            [0 1 2*dt_stamps(i+1) 3*dt_stamps(i+1)^2 0 -1 -2*dt_stamps(i+1) -3*dt_stamps(i+1)^2];
        A(length(distances)*2+2+i+length(distances)-1,(i-1)*4+1:(i-1)*4+8) = ...
            [0 0 2 6*dt_stamps(i+1) 0 0 -2 -6*dt_stamps(i+1)];
    end
    C=A\X;
    t_idx = 1;
end

if t < dt_stamps(end)
    if t >= dt_stamps(t_idx)
        t_idx = t_idx+1;
    end
    S = [1 t t^2 t^3;...
        0 1 2*t 3*t^2;...
        0 0 2 6*t]*...
        C((t_idx-2)*4+1:(t_idx-2)*4+4,:);
    pos = S(1,:)';
    vel = S(2,:)';
    acc = S(3,:)';
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