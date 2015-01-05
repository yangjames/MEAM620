function [path, cost] = shortestpath(Graph, start, goal)
% SHORTESTPATH Find the shortest path from start to goal on the given Graph.
%   PATH = SHORTESTPATH(Graph, start, goal) returns an M-by-1 matrix, where each row
%   consists of the node on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-1 matrix.
%
%   [PATH, COST] = SHORTESTPATH(...) returns the path as well as
%   the total cost of the path.
%   Arguments:
%      Graph is a N-by-3 matrix, representing all edges in the given graph.
%      Each row has the format of [U, V, W] where U and V are two nodes that
%      this edge connects and W is the weight(cost) of traveling through
%      the edge. For example, [1 5 20.2] means this edge connects
%      node 1 and 5 with cost 20.2. Please note that all edges are
%      undirected and you can assume that the nodes in Graph are indexed by
%      1, ..., n where n is the number of nodes in the Graph.
%
% Hint: You may consider constructing a different graph structure to speed up
% you code.

%% initialize graph as an empty path with no cost
path = zeros(0,1);
cost = 0;

%% special case with an empty graph
if isempty(Graph)
    return;
end

%% special case when we start at our goal
if start == goal
    path = start;
    cost = 0;
    return;
end

%% filter out repeat edges and minimize multiple paths
sorted_graph = sortrows([sort(Graph(:,1:2),2) Graph(:,3)]);
filtered_graph = sorted_graph(1,:);
for i = 2:size(sorted_graph,1)
    if logical(ismember(sorted_graph(i,1:2),filtered_graph(end,1:2),'rows'))
        filtered_graph(end,3) = min([sorted_graph(i,3) filtered_graph(end,3)]);
    else
        filtered_graph(end+1,:) = sorted_graph(i,:);
    end
end
%{
left_nodes = unique(sorted_graph(:,1));
right_nodes = unique(sorted_graph(:,2));
filtered_graph = zeros(size(sorted_graph));
for i = 1:length(left_nodes)
    idx_l = sorted_graph(:,1) == i;
    for j = 1:length(right_nodes)
        idx_r = sorted_graph(idx_l,2) == j;
        
    end
end
%}
%{
not_checked = true(size(Graph,1),1);
new_graph = zeros(size(Graph));
iteration = 1;
Graph_nodes = Graph(:,1:2);
Graph_nodes_t = fliplr(Graph_nodes);
while sum(not_checked)
    test_edges = Graph_nodes(not_checked,:);
    test_edge = test_edges(1,:);
    repeats = (ismember(Graph_nodes, test_edge, 'rows') | ismember(Graph_nodes_t, test_edge, 'rows')) & not_checked;
    val = min(Graph(repeats,3));
    new_graph(iteration,:) = [test_edge, val];
    iteration = iteration+1;
    not_checked(repeats) = false;
end
new_graph(iteration:end,:) = [];
%}

%% restructure the graph as a sparse symmetric adjacency matrix
graph = sparse(filtered_graph(:,1),filtered_graph(:,2),filtered_graph(:,3));
%graph = sparse(Graph(:,1), Graph(:,2), Graph(:,3));
[n,m] = size(graph);
if n < m
    graph = vertcat(graph,zeros(m-n,m));
    n = m;
else if m < n
        graph = horzcat(graph,zeros(n,n-m));
    end
end
graph = graph + graph';

distance = Inf(n,1);
previous = NaN(n,1);
unvisited = sparse(true(n,1));
distance(start) = 0;

%% loop until goal is visited or is determined to be unreachable
while unvisited(goal) && min(distance(unvisited)) ~= Inf
    % get unvisited node with smallest distance
    [dist, current_node] = min(distance./unvisited);
        
    % obtain tentative distances and update distances if necessary
    temp_dist = graph(:, current_node);
    nonzero = (temp_dist ~= 0);
    if sum(nonzero)
        tentative = logical((dist + temp_dist < distance).*nonzero.*unvisited);
        distance(tentative) = dist + temp_dist(tentative);
        previous(tentative) = current_node;
    end
    
    % remove current node from graph
    unvisited(current_node) = false;
end

%% find the path if it exists
cost = distance(goal);
if cost ~= Inf
    path = NaN(size(distance));
    iterator = length(path);
    path_node = goal;
    while path_node ~= start
        path(iterator) = path_node;
        iterator = iterator - 1;
        path_node = previous(path_node);
    end
    path(iterator) = start;
    path(1:iterator-1) = [];
end