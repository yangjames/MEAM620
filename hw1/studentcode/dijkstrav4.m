function [path, num_expanded]=dijkstrav4(map,start,goal,astar)
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
previous = NaN(num_nodes,1);

% heap indices
node_heap = (1:num_nodes)';

% minimize the heap
node_heap(1) = start_node;
node_heap(start_node) = 1;
node_heap_idx =node_heap;
node_heap_len = length(node_heap);

% heuristic initialization
node_coords = node_to_xyz(map,(1:num_nodes)');
if astar
    heuristic = sqrt(sum(bsxfun(@minus,node_coords,goal).^2,2));
else
    heuristic = sparse(zeros(num_nodes,1));
end

cost = Inf(num_nodes,1);
cost(start_node) = 0;
f_cost = Inf(num_nodes,1);
f_cost(start_node) = heuristic(start_node);

%% initialize 26-connected neighbors coordinates and distances

neighbors_6 = [0 0 -map.z_res;...
                0 -map.xy_res 0;...
                -map.xy_res 0 0;...
                map.xy_res 0 0;...
                0 map.xy_res 0;...
                0 0 map.z_res];
neighbors_dist = sqrt(sum(neighbors_6.^2,2));

%{
neighbors_26 = [repmat([-map.xy_res; 0; map.xy_res],9,1)...
    repmat([repmat(-map.xy_res,3,1); zeros(3,1); repmat(map.xy_res,3,1)],3,1)...
    [repmat(-map.z_res,9,1); zeros(9,1); repmat(map.z_res,9,1)]];
neighbors_dist = sqrt(sum(neighbors_26.^2,2));
neighbors_26(14,:) = [];
neighbors_dist(14) = [];
%}
%{
%% plotting stuff
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
h1 = plot3(0,0,0,'y.');

plot_path(map,path);
%}
%% loop until completion
while node_heap(1) ~= goal_node
    % get min
    if f_cost(node_heap(1)) == Inf
        return;
    end
    current_node = node_heap(1);
    %visited(current_node) = true;
    
    % remove current node from heap
    node_heap_idx(node_heap(1)) = node_heap_len;
    node_heap_idx(node_heap(node_heap_len)) = NaN;
    node_heap(1) = node_heap(node_heap_len);
    node_heap_len = node_heap_len - 1;
    
    % sort heap
    idx = 1;
    while idx < node_heap_len
        left = idx*2;
        right = idx*2+1;
        smallest = idx;
        if left <= node_heap_len && f_cost(node_heap(left)) < f_cost(node_heap(smallest))
            smallest = left;
        end
        if right <= node_heap_len && f_cost(node_heap(right)) < f_cost(node_heap(smallest))
            smallest = right;
        end
        if smallest ~= idx
            node_heap_idx(node_heap(smallest)) = idx;
            node_heap_idx(node_heap(idx)) = smallest;
            temp = node_heap(idx);
            node_heap(idx) = node_heap(smallest);
            node_heap(smallest) = temp;
            idx = smallest;
        else
            break;
        end
    end
    
    % obtain neighbors
    current_coord = node_to_xyz(map,current_node);
    neighbors_coord = bsxfun(@plus,neighbors_6,current_coord);
    c = ~collide(map,neighbors_coord);
    
    neighbors_nodes = xyz_to_node(map,neighbors_coord(c,:));
    filtered_dist = neighbors_dist(c);
    for i = 1:length(neighbors_nodes)
        if cost(current_node) + filtered_dist(i) < cost(neighbors_nodes(i))
            cost(neighbors_nodes(i)) = cost(current_node) + filtered_dist(i);
            f_cost(neighbors_nodes(i)) = cost(neighbors_nodes(i)) + heuristic(neighbors_nodes(i));
            previous(neighbors_nodes(i)) = current_node;

            idx = node_heap_idx(neighbors_nodes(i));
            while idx ~= 1
                parent = floor(idx/2);
                if f_cost(node_heap(parent)) > f_cost(node_heap(idx))
                    node_heap_idx(neighbors_nodes(i)) = parent;
                    node_heap_idx(node_heap(parent)) = idx;
                    temp = node_heap(idx);
                    node_heap(idx) = node_heap(parent);
                    node_heap(parent) = temp;
                    idx = parent;
                else
                    break;
                end
            end
        end
    end
    
    %{
    %% more plotting stuff
    if ~mod(num_nodes-node_heap_len,100)
        coord = node_to_xyz(map,node_heap(~isinf(cost(node_heap(1:node_heap_len)))));
        set(h1,'XData',coord(:,1),'YData',coord(:,2),'ZData',coord(:,3));
    end
    drawnow
    %}
end

%% find the path if it exists
goal_cost = cost(goal_node);
if goal_cost ~= Inf
    node_path = NaN(size(cost));
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
num_expanded = num_nodes-node_heap_len;
%plot_path(map,path);
end
