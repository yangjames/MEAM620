function [ desired_state ] = trajectory_generator(t, qn, map, path)
persistent path0 c_time dt_stamps t_idx t_total pivot_idx C path1 truncated_path
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
    path1 = path{1};
    
    % filter the path
    path0 = path1;
    
    % find indices where path switches directions
    pivot_idx = find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;
    
    truncated_path = path0;%[path0(1,:); path0(pivot_idx,:); path0(end,:)];

    % get rid of zig-zags
    minimized_flag = false;
    while ~minimized_flag
        for i = 1:length(truncated_path)-2
            num_points = sum((truncated_path(i+2,:)-truncated_path(i,1)).^2,2)/(min(map.xy_res,map.z_res)/10);
            x_space = linspace(truncated_path(i,1),truncated_path(i+2,1),num_points);
            y_space = linspace(truncated_path(i,2),truncated_path(i+2,2),num_points);
            z_space = linspace(truncated_path(i,3),truncated_path(i+2,3),num_points);
            if ~any(collide(map,[x_space' y_space' z_space']));
                truncated_path(i+1,:) = [];
                minimized_flag = false;
                break;
            else
                minimized_flag = true;
            end
        end
    end
    
    % calculate distance between pivot points
    distances = sqrt(sum((truncated_path(1:end-1,:)-truncated_path(2:end,:)).^2,2));
    
    % gather time stamps between pivot points
    dt_stamps = cumsum([0;sqrt(distances)*1.05]);
    t_idx = 1;
end

if t < dt_stamps(end)
    if t >= dt_stamps(t_idx)
        t_idx = t_idx+1;
    end
    dt = t-dt_stamps(t_idx-1);
    X_0 = [truncated_path(t_idx-1,:);...
        0 0 0;...
        0 0 0;...
        truncated_path(t_idx,:);...
        0 0 0;...
        0 0 0];
    a = get_interp_weights(X_0,dt_stamps(t_idx)-dt_stamps(t_idx-1));
    X = [1 dt dt^2 dt^3 dt^4 dt^5;...
        0 1 2*dt 3*dt^2 4*dt^3 5*dt^4;...
        0 0 2 6*dt 12*dt^2 20*dt^3]*a;
    pos = X(1,:)';
    vel = X(2,:)';
    acc = X(3,:)';
else
    pos = truncated_path(end,:)';
    vel = [0 0 0]';
    acc = [0 0 0]';
end
%}
%{d
if isempty(C)
    path1 = path{1};

    % filter the path
    path0 = path1;
    
    % find indices where path switches directions
    pivot_idx = find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;

    truncated_path = [path0(1,:); path0(pivot_idx,:); path0(end,:)];
    
    %% get rid of zig-zags
    minimized_flag = false;
    while ~minimized_flag
        for i = 1:length(truncated_path)-2
            num_points = sum((truncated_path(i+2,:)-truncated_path(i,1)).^2,2)/(min(map.xy_res,map.z_res)/10);
            x_space = linspace(truncated_path(i,1),truncated_path(i+2,1),num_points);
            y_space = linspace(truncated_path(i,2),truncated_path(i+2,2),num_points);
            z_space = linspace(truncated_path(i,3),truncated_path(i+2,3),num_points);
            if ~any(collide(map,[x_space' y_space' z_space']));
                truncated_path(i+1,:) = [];
                minimized_flag = false;
                break;
            else
                minimized_flag = true;
            end
        end
    end
    
    % calculate distance between pivot points
    distances = sqrt(sum((truncated_path(1:end-1,:)-truncated_path(2:end,:)).^2,2));
    
    % gather time stamps between pivot points
    dt_stamps = cumsum([0;sqrt(distances)*0.7]);
    
    % fit spline
    C = fit_cubic_spline(truncated_path, dt_stamps);
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
    pos = truncated_path(end,:)';
    vel = [0 0 0]';
    acc = [0 0 0]';
end
%}
%pos = path1(1,:)';
%vel = [0 0 0]';
%acc = [0 0 0]';

%{
if t < 7
    pos = path1(1,:)';
    vel = [0 0 0]';
    acc = [0 0 0]';
else
    pos = path1(1,:)'+[10 5 5]';
    vel = [0 0 0]';
    acc = [0 0 0]';
end
%}
yaw = 0;
yawdot = 0;

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;