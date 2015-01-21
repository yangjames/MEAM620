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
num_nodes = num_depth*num_plane

%% define key nodes
start_node = xyz_to_node(map,start)
goal_node = xyz_to_node(map,goal)

%% initialize storage variables and heuristic
% heap indices
node_heap = (1:num_nodes)';

% get collisions
node_coords = node_to_xyz(map,node_heap);
collisions = ~collide(map, node_coords);

% heuristic initialization
if astar
    heuristic = sqrt(sum(bsxfun(@minus,node_coords,goal).^2,2));
else
    heuristic = sparse(zeros(num_nodes,1));
end

% heap initialization
node_heap(1) = start_node;
node_heap(start_node) = 1;
sorted_collisions = collisions(node_heap);
node_heap = node_heap(sorted_collisions);
heap_len = length(node_heap);
index_list = NaN(num_nodes,1);
for i = 1:heap_len
    index_list(node_heap(i)) = i;
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

%% plotting stuff
figure(2)
visited = ~collisions;
clf
h2 = plot3(start(1),start(2),start(3),'g*');
hold on
grid on
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
xlim(map.boundary([1,4]))
ylim(map.boundary([2,5]))
zlim(map.boundary([3,6]))
h3 = plot3(goal(1),goal(2),goal(3),'r*');
h1 = plot3(0,0,0,'y.');

plot_path(map,path);

%% loop until algorithm is complete
while node_heap(1) ~= goal_node
    % obtain minimum node and check exit condition
    if f_score(node_heap(1)) == Inf
        return;
    end
    current_node = node_heap(1);
    visited(current_node) = current_node;
    
    % remove current node from heap
    index_list(node_heap(1)) = heap_len;
    index_list(node_heap(heap_len)) = NaN;
    node_heap(1) = node_heap(heap_len);
    heap_len = heap_len - 1;
    
    % sort the heap
    idx = 1;
    while idx < heap_len
        left = idx*2;
        right = idx*2+1;
        smallest = idx;
        if left <= heap_len && f_score(node_heap(left)) < f_score(node_heap(smallest))
            smallest = left;
        end
        if right <= heap_len && f_score(node_heap(right)) < f_score(node_heap(smallest))
            smallest = right;
        end
        if smallest ~= idx
            temp = node_heap(idx);
            node_heap(idx) = node_heap(smallest);
            node_heap(smallest) = temp;
            index_list(node_heap(idx)) = smallest;
            index_list(node_heap(smallest)) = idx;
            idx = smallest;
        else
            break;
        end
    end
    
    % get neighbors
    current_coord = node_to_xyz(map,current_node);
    neighbors_coord = bsxfun(@plus,neighbors_26,current_coord);
    c = ~collide(map,neighbors_coord);
    
    % obtain tentative distances and update distances if necessary
    if sum(c)
        neighbors_nodes = xyz_to_node(map,neighbors_coord(c,:));
        filtered_dist = neighbors_dist(c);
        for i = 1:length(neighbors_nodes)
            if g_score(current_node) + neighbors_dist(i) < g_score(neighbors_nodes(i))
                g_score(neighbors_nodes(i)) = g_score(current_node) + filtered_dist(i);
                f_score(neighbors_nodes(i)) = g_score(neighbors_nodes(i)) + heuristic(neighbors_nodes(i));
                previous(neighbors_nodes(i)) = current_node;
                
                % sort the heap
                idx = index_list(neighbors_nodes(i));
                while idx ~= 1
                    parent = floor(idx/2);
                    if f_score(node_heap(parent)) > f_score(node_heap(idx))
                        temp = node_heap(idx);
                        node_heap(idx) = node_heap(parent);
                        node_heap(parent) = temp;
                        index_list(neighbors_nodes(i)) = parent;
                        index_list(node_heap(parent)) = idx;
                        idx = parent;
                    else
                        break;
                    end
                end
            end
        end
    end
    
    %% more plotting stuff
    if ~mod(num_nodes-heap_len,100)
        coord = node_to_xyz(map,node_heap(~isinf(g_score(node_heap(1:heap_len)))));
        set(h1,'XData',coord(:,1),'YData',coord(:,2),'ZData',coord(:,3));
    end
    drawnow
end

%% find the path if it exists
cost = g_score(goal_node);
if cost ~= Inf
    node_path = NaN(size(node_heap));
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
end
num_expanded = num_nodes-heap_len;
plot_path(map,path);
end