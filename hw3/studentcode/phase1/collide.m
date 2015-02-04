function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

%% get true bounds
margin = [map.margin map.margin map.margin -map.margin -map.margin -map.margin]*101/100;
n_bound = map.boundary;%+[-map.xy_res -map.xy_res -map.z_res map.xy_res map.xy_res map.z_res]*1/100;% + margin;
n_blocks = repmat(bsxfun(@minus, map.blocks(:,1:6), margin),size(points,1),1);

%% get points that are out of bounds
ob = sum(bsxfun(@gt, n_bound(1:3), points) | bsxfun(@lt, n_bound(4:6), points),2)>0;

%% get points that collide with an object
idx = reshape(meshgrid(1:size(points,1),1:size(map.blocks,1)),size(map.blocks,1)*size(points,1),1);
inside = sum(bsxfun(@le, n_blocks(:,1:3),points(idx,:)) & bsxfun(@ge, n_blocks(:,4:6),points(idx,:)),2) == 3;
co = sum(reshape(inside',size(map.blocks,1),size(points,1)))'>0;
C = co|ob;
end