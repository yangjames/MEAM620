function [path, num_expanded] = dijkstra_v2(map, start, goal, astar)
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
path = zeros(0,3);
num_expanded = 0;
if isempty(map.boundary) || collide(map,start) || collide(map,goal)
    return;
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

%% initialize storage variables and heuristic
% heap indices
nodes = (1:num_nodes)';
nodes(1) = start_node;
nodes(start_node) = 1;
index_list = nodes;

% heuristic initialization
if astar
    heuristic = sum(bsxfun(@minus,node_to_xyz(map,nodes),goal).^2,2);
else
    heuristic = sparse(zeros(size(nodes)));
end

% actual cost
g_score = Inf(num_nodes,1);
g_score(start_node) = 0;

% previously visited node
previous = NaN(num_nodes,1);

% heuristic cost
f_score = Inf(num_nodes,1);
f_score(start_node) = heuristic(start_node);

%% initialize 26-connected neighbors coordinates and distances
neighbors_26 = [repmat([-map.xy_res; 0; map.xy_res],9,1)...
    repmat([repmat(-map.xy_res,3,1); zeros(3,1); repmat(map.xy_res,3,1)],3,1)...
    [repmat(-map.z_res,9,1); zeros(9,1); repmat(map.z_res,9,1)]];
neighbors_dist = sqrt(sum(neighbors_26.^2,2));
neighbors_26(14,:) = [];
neighbors_dist(14) = [];

%% loop until algorithm is complete
while nodes(1) ~= goal_node
    % obtain minimum and check exit condition
    if f_score(nodes(1)) == Inf
        return;
    end
    current_node = nodes(1);
    
    % get neighbors
    current_coord = node_to_xyz(map,current_node);
    neighbors_coord = bsxfun(@plus,neighbors_26,current_coord);
    c = ~collide(map,neighbors_coord);
    
    % obtain tentative distances and update distances if necessary
    if sum(c)
        neighbors_nodes = xyz_to_node(map,neighbors_coord(c,:));
        for i = 1:length(neighbors_nodes)
            if g_score(current_node) + neighbors_dist(i) < g_score(neighbors_nodes(i))
                g_score(neighbors_nodes(i)) = g_score(current_node) + neighbors_dist(i);
                f_score(neighbors_nodes(i)) = g_score(neighbors_nodes(i)) + heuristic(neighbors_nodes(i));
                
                % sort the heap
                done = 0;
                idx = index_list(neighbors_nodes(i));
                while idx ~= 1
                    level_up = floor(idx/2);
                    if g_score(nodes(level_up)) < g_score(nodes(idx))
                        temp = nodes(idx);
                        nodes(idx) = nodes(level_up);
                        nodes(level_up) = temp;
                        index_list(neighbors_nodes(i)) = level_up;
                        index_list(nodes(level_up)) = idx;
                        idx = level_up;
                    else
                        done = 1;
                    end
                end
                previous(neighbors_nodes(i)) = current_node;
            end
        end
    end
    
    % remove current node from heap
    index_list(nodes(heap_len)) = NaN;
    nodes(1) = nodes(heap_len);
    heap_len = heap_len - 1;
    
    % sort the heap
    idx = 1;
    while idx <= heap_len
        left = idx*2;
        right = idx*2+1;
        smallest = idx;
        if left <= heap_len && g_score(nodes(left)) < g_score(nodes(smallest))
            smallest = left;
        end
        if right <= heap_len && g_score(nodes(right)) < g_score(nodes(smallest))
            smallest = right;
        end
        if smallest ~= idx
            temp = nodes(idx);
            nodes(idx) = nodes(smallest);
            nodes(smallest) = temp;
            index_list(nodes(idx)) = smallest;
            index_list(nodes(smallest)) = idx;
            idx = smallest;
        else
            break;
        end
    end
end

%% find the path if it exists
cost = heap(goal_node);
if cost ~= Inf
    node_path = NaN(size(heap));
    iterator = length(node_path);
    path_node = goal_node;
    while path_node ~= start_node
        node_path(iterator) = path_node;
        iterator = iterator - 1;
        path_node = previous(path_node);
    end
    node_path(iterator) = start_node;
    node_path(1:iterator-1) = [];
    path = node_to_xyz(map,node_path);
    path(1,:) = start;
    path(end,:) = goal;
    num_expanded = sum(~unvisited);
end
plot_path(map,path);
end