function [pos, eul] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   eul - 3x1 euler angles of the quadrotor
K=[314.1779 0         199.4848; ...
    0         314.2218  113.7838; ...
    0         0         1];

p4_xy = [2*mod(sensor.id,12)*0.152;floor(sensor.id./12)*2*0.152];
p4_xy(2,sensor.id>35) = p4_xy(2,sensor.id>35) + 0.026;
p4_xy(2,sensor.id>71) = p4_xy(2,sensor.id>71) + 0.026;

p0_xy = bsxfun(@plus,p4_xy,[0.152;0.152]/2);
p3_xy = bsxfun(@plus,p4_xy,[0; 0.152]);
p2_xy = bsxfun(@plus,p4_xy,[0.152; 0.152]);
p1_xy = bsxfun(@plus,p4_xy,[0.152; 0]);

p_xy = [p0_xy p1_xy p2_xy p3_xy p4_xy];
p_im = [sensor.p0 sensor.p1 sensor.p2 sensor.p3 sensor.p4];

H = est_homography(p_im(1,:),p_im(2,:),p_xy(1,:),p_xy(2,:));
H_w = K\H;
[U,~,V] = svd([H_w(:,1:2) cross(H_w(:,1),H_w(:,2))]);
R_cam = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

XYZ = [-0.04, 0.0, -0.03]';
pos = R_cam'*(XYZ-H_w(:,3)/norm(H_w(:,1)));

R_cam = [1 0 0; 0 -1 0; 0 0 -1]*[cos(pi/4) -sin(pi/4) 0; sin(pi/4) cos(pi/4) 0; 0 0 1]*R_cam;
eul = -[asin(R_cam(3,2));atan2(-R_cam(3,1),R_cam(3,3));atan2(-R_cam(1,2),R_cam(2,2))];
end
