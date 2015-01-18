function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

%% check if points are inside the boundaries
collisions = bsxfun(@plus,[map.boundary; map.blocks],...
    [map.margin map.margin map.margin -map.margin -map.margin -map.margin]);
C = sum([bsxfun(@geq, collisions(:,1:3), points) bsxfun(@leq, collisions(:,4:6), points)],2) > 0;
end
