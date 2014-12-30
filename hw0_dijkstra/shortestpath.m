function [path, cost] = shortestpath(Graph, start, goal)
profile on
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

path = zeros(0,1);
cost = Inf;

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

%graph = sparse(Graph(:,1),Graph(:,2),Graph(:,3));
dim = max(max(Graph(:,1)),max(Graph(:,2)));
idx = sub2ind([dim dim],Graph(:,1), Graph(:,2));
graph = Inf(dim);
graph(idx) = Graph(:,3);
[n,m] = size(graph);
if n < m
    graph = vertcat(graph,zeros(m-n,m));
else
    if m > n
        graph = horzcat(graph,zeros(n-m,n));
    end
end
graph = min(graph,graph');
[n,~] = size(graph);

distance = Inf(n,1);
previous = NaN(n,1);
unvisited = true(n,1);
distance(start) = 0;

while unvisited(goal) && min(distance(unvisited)) ~= Inf
    % get unvisited node with smallest distance
    [dist, current_node] = min(distance./unvisited);
        
    % obtain tentative distances and update distances if necessary
    temp_dist = graph(:,current_node);
    tentative = dist + temp_dist.*unvisited < distance.*unvisited;
    distance(tentative) = dist + temp_dist(tentative);
    previous(tentative) = current_node;

    % remove current node from graph
    unvisited(current_node) = false;
end
cost = distance(goal);

%% find the path if it exists
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
     if iterator < length(path)
        path(1:iterator-1) = [];
     end
end