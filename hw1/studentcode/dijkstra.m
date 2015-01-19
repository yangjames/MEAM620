function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end

if isempty(map.boundary) || collide(map,start) || collide(map,goal)
    path = [];
    num_expanded = 0;
end

%% define map characteristics
num_rows = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_cols = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_depth = floor((map.boundary(6)-map.boundary(3))/map.z_res)+1;
num_plane = num_rows*num_cols;
num_nodes = num_depth*num_plane;

%% define key nodes
start_node = xyz_to_node(map,start);
goal_node = xyz_to_node(map,goal);
first_node = xyz_to_node(map,map.boundary(1:3));
last_node = xyz_to_node(map,map.boundary(4:6));

%% initialize storage variables
nodes = (first_node:last_node)';
distance = [Inf(num_nodes,1) nodes];
previous = NaN(num_nodes,1);
unvisited = sparse(true(num_nodes,1));
unvisited_full = full(unvisited);
distance(1,:) = [0 start_node];
distance(start_node-first_node+1,:) = [Inf 1];

neighbors_26 = [repmat([-map.xy_res;0;map.xy_res],9,1)...
    repmat([repmat(-map.xy_res,3,1);repmat(0,3,1);repmat(map.xy_res,3,1)],3,1)...
    [repmat(-map.z_res,9,1);repmat(0,9,1);repmat(map.z_res,9,1)]];
neighbors_dist = sqrt(sum(neighbors_26.^2,2));
neighbors_26(14,:) = [];
neighbors_dist(14) = [];
            
% loop until algorithm is complete
while ~unvisited_full(goal_node-first_node+1)
    % get current node
    dist = distance(1,1);
        
    if dist == Inf
       break;
    end
    current_node = distance(1,2);
    
    current_coord = node_to_xyz(map,current_node);
    neighbors_coord = bsxfun(@plus,neighbors_26,current_coord);
    neighbors_nodes = xyz_to_node(map,neighbors_coord);
    
end