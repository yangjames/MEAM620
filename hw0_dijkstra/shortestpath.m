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
filtered_graph = zeros(size(sorted_graph));
filtered_graph(1,:) = sorted_graph(1,:);
iterator = 1;
for i = 2:size(sorted_graph,1)
    if logical(ismember(sorted_graph(i,1:2),filtered_graph(iterator,1:2),'rows'))
        filtered_graph(iterator,3) = min([sorted_graph(i,3) filtered_graph(iterator,3)]);
    else
        iterator = iterator+1;
        filtered_graph(iterator,:) = sorted_graph(i,:);
    end
end
if iterator < size(sorted_graph,1)
    filtered_graph(iterator+1:end,:) = [];
end

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
unvisited = sparse(true(n,1));
distance(start) = 0;

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