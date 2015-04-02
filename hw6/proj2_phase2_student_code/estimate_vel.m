function [vel, omg] = estimate_vel(sensor, varargin)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
%          - rpy, omg, acc: imu readings, you should not use these in this phase
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
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor
persistent oldPoints t_filtered t_filtered_prev RC pointTracker K alpha

%% intialize
if isempty(sensor.id)
    vel = [];
    omg = [];
    return
end
if isempty(oldPoints)
    vel = [];
    omg = [];
    K=[314.1779 0         199.4848; ...
        0         314.2218  113.7838; ...
        0         0         1];
    RC = 10;
    alpha = 1/(RC+1);
    
    t_filtered_prev = 0;
    t_filtered = alpha*sensor.t + (1-alpha)*t_filtered_prev;
    t_filtered_prev = t_filtered;
    oldPoints = corner(sensor.img);
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
    initialize(pointTracker, oldPoints, sensor.img);
    return
end

%% filter time and calculate dt
t_filtered = alpha*sensor.t + (1-alpha)*t_filtered_prev;
dt_filtered = t_filtered - t_filtered_prev;

%% track points
[points, isFound] = step(pointTracker, sensor.img);
visiblePoints = points(isFound, :);
oldInliers = oldPoints(isFound, :);

if size(visiblePoints, 1) >= 2 % need at least 2 points
    %% calculate pose
    p4_xy = [mod(sensor.id,12);...
            floor(sensor.id./12)]*2*0.152;
    p4_xy(2,sensor.id>35) = p4_xy(2,sensor.id>35) + 0.026;
    p4_xy(2,sensor.id>71) = p4_xy(2,sensor.id>71) + 0.026;

    p0_xy = bsxfun(@plus,p4_xy,[0.152;0.152]/2);
    p3_xy = bsxfun(@plus,p4_xy,[0; 0.152]);
    p2_xy = bsxfun(@plus,p4_xy,[0.152; 0.152]);
    p1_xy = bsxfun(@plus,p4_xy,[0.152; 0]);

    p_xy = [p0_xy p1_xy p2_xy p3_xy p4_xy];
    p_im = [sensor.p0 sensor.p1 sensor.p2 sensor.p3 sensor.p4];

    %% estimate the homography
    A = zeros(size(p_xy,2)*2,9);

    for i = 1:size(p_xy,2),
        a = [p_xy(:,i)',1];
        b = [0 0 0];
        c = p_im(:,i);
        d = -c*a;
        A((i-1)*2+1:(i-1)*2+2,1:9) = [[a b;b a] d];
    end

    [~, ~, V] = svd(A);
    h = V(:,9);
    H = reshape(h,3,3)';

    H = H/H(3, 3);
    H_w = K\H;
    [U,~,V] = svd([H_w(:,1:2) cross(H_w(:,1),H_w(:,2))]);
    R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

    pos = H_w(:,3)/norm(H_w(:,1));
    
    % match features
    [~, oldInliers, visiblePoints] = estimateGeometricTransform(...
        oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

    % get scale factor
    n = size(visiblePoints,1);
    p_current = K\[visiblePoints'; ones(1,n)];
    lambda = (R(:,3)'*pos)./(R(:,3)'*p_current);
    
    % calculate optical flow
    p_prev = K\[oldInliers';ones(1,n)];
    %d_pixels = K\[(visiblePoints-oldInliers)'; zeros(1,n)]/dt_filtered;
    d_pixels = (p_current-p_prev)/dt_filtered;
    p_dot = reshape(d_pixels(1:2,:), n*2,1);

    % get optical flow to velocities transformation matrix
    A = zeros(n*2,6);
    for i = 1:length(lambda)
        A(2*(i-1)+1:2*i,:) = [-1/lambda(i) 0 p_current(1,i)/lambda(i) prod(p_current(1:2,i)) -(1+p_current(1,i)^2) p_current(2,i);...
                                0 -1/lambda(i) p_current(2,i)/lambda(i) (1+p_current(2,i)^2) -prod(p_current(1:2,i)) -p_current(1,i)];
    end
    
    % calculate velocities
    vals = A\p_dot;
    vel = R'*vals(1:3);
    omg = R'*vals(4:6);
    
    % propagate the matched features
    if size(visiblePoints,1)<150
        oldPoints = corner(sensor.img);
    else
        oldPoints = visiblePoints;
    end
    setPoints(pointTracker, oldPoints);
else
    vel = [];
    omg = [];
end

%% propagate persistent variables
t_filtered_prev = t_filtered;
end