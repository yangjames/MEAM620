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

if nargin ~= 3
    error('Incorrect number of arguments');
end

path = zeros(0,1);
cost = Inf;

%% special case with an empty graph
if isempty(Graph)
    return;
end

nodes = unique(Graph(:,1:2))

%% special case when start or goal node does not exist
if sum(start == nodes) == 0 || sum(goal == nodes) == 0
    error('Invalid start or goal node');
end

%% special case when we start at our goal
if start == goal
    path = start;
    cost = 0;
    return;
end

distance = ones(size(nodes))*Inf;
previous = ones(size(nodes))*NaN;
unvisited = logical(ones(size(nodes)));

distance(nodes == start) = 0;

%while unvisited(nodes == start)
    
%end