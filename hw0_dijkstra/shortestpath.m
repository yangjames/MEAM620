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

%% prune off repeat edges and minimize multiple paths
sorted_graph = sortrows([sort(Graph(:,1:2),2) Graph(:,3)]);
labeled_edges = 10*sorted_graph(:,1)+sorted_graph(:,2);
filtered_graph = sorted_graph([true; labeled_edges(2:end) ~= labeled_edges(1:end-1)],:);

%% restructure the graph as a sparse symmetric adjacency matrix
graph = sparse(filtered_graph(:,1),filtered_graph(:,2),filtered_graph(:,3));
[n,m] = size(graph);
if n < m
    graph = vertcat(graph,zeros(m-n,m));
    n = m;
else if m < n
        graph = horzcat(graph,zeros(n,n-m));
    end
end
graph = graph + graph';

%% loop until goal is visited or is determined to be unreachable
distance = Inf(n,1);
previous = NaN(n,1);
nodes = (1:n)';
unvisited = sparse(true(n,1));
unvisited_full = full(unvisited);
distance(start) = 0;

while unvisited_full(goal) && min(distance(unvisited)) ~= Inf
    % get unvisited node with smallest distance
    [dist,idx] = min(distance(unvisited));
    temp = nodes(unvisited);
    current_node = temp(idx);
    
    % obtain tentative distances and update distances if necessary
    temp_dist = graph(:, current_node);
    nonzero = (temp_dist ~= 0);
    if sum(nonzero)
        tentative = ((dist + temp_dist < distance) & nonzero) & unvisited_full;
        distance(tentative) = dist + temp_dist(tentative);
        previous(tentative) = current_node;
    end
    
    % remove current node from graph
    unvisited(current_node,1) = false;
    unvisited_full(current_node) = false;
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