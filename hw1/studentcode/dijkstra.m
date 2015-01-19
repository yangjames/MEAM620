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

%% initialize storage variables
nodes = (1:num_nodes)';
distance = Inf(num_nodes,1);
previous = NaN(num_nodes,1);
unvisited = sparse(true(num_nodes,1));
unvisited_full = full(unvisited);
distance(start_node,:) = 0;

%% initialize 26-connected neighbors coordinates and distances
neighbors_26 = [repmat([-map.xy_res; 0; map.xy_res],9,1)...
    repmat([repmat(-map.xy_res,3,1); zeros(3,1); repmat(map.xy_res,3,1)],3,1)...
    [repmat(-map.z_res,9,1); zeros(9,1); repmat(map.z_res,9,1)]];
neighbors_dist = sqrt(sum(neighbors_26.^2,2));
neighbors_26(14,:) = [];
neighbors_dist(14) = [];

figure(2)
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
h1 = plot3(0,0,0,'k*');
h4 = plot3(0,0,0,'b*');

%% loop until algorithm is complete
while unvisited_full(goal_node)
    %{
    iterator = 1;
    coord = node_to_xyz(map,iterator);
    h5 = plot3(coord(1),coord(2),coord(3),'k*');
    while(iterator < num_nodes)
        iterator = iterator+1;
        coord = node_to_xyz(map,iterator);
        set(h5,'XData',coord(1),'YData',coord(2),'ZData',coord(3));
    end
    %}
    % get unvisited node with smallest distance
    [dist,idx] = min(distance(unvisited));
    if dist == Inf
        break;
    end
    temp = nodes(unvisited);
    current_node = temp(idx);
        
    % obtain neighbors
    current_coord = node_to_xyz(map,current_node);
    neighbors_coord = bsxfun(@plus,neighbors_26,current_coord);
    neighbors_nodes = xyz_to_node(map,neighbors_coord);
    c = ~collide(map,neighbors_coord) & unvisited_full(neighbors_nodes);
    
    % obtain tentative distances and update distances if necessary
    if astar
        heuristic = sqrt(sum(bsxfun(@minus,neighbors_coord,goal).^2,2));
    else
        heuristic = 0;
    end
    tentative = (dist + neighbors_dist + heuristic < distance(neighbors_nodes)) & c;
    distance(neighbors_nodes(tentative)) = dist + neighbors_dist(tentative) + heuristic(tentative);
    previous(neighbors_nodes(tentative)) = current_node;
    
    % remove current node from graph
    unvisited(current_node,1) = false;
    unvisited_full(current_node) = false;
    
    coord = node_to_xyz(map,find(~unvisited_full));
    figure(1)
    set(h1,'XData',coord(:,1),'YData',coord(:,2),'ZData',coord(:,3));
    set(h4,'Xdata',neighbors_coord(:,1),'YData',neighbors_coord(:,2),'ZData',neighbors_coord(:,3));
    drawnow
end

%% find the path if it exists
cost = distance(goal_node);
if cost ~= Inf
    node_path = NaN(size(distance));
    iterator = length(node_path);
    path_node = goal_node;
    while path_node ~= start_node
        node_path(iterator) = path_node;
        iterator = iterator - 1;
        path_node = previous(path_node);
    end
    node_path(iterator) = start_node;
    node_path(1:iterator-1) = [];
end
path = node_to_xyz(map,node_path);
num_expanded = sum(~unvisited);
plot_path(map,path);
end