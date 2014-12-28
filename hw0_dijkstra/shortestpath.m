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

%% initialize graph
nodes = unique(Graph(:,1:2))';
distances(nodes ~= start) = Inf;
reference(nodes ~= start) = nan;
reference(nodes == start) = start;
unvisited = ones(size(nodes));

path = [];
cost = Inf;
%% loop through all the nodes
while unvisited(nodes == goal) || min(distances(logical(unvisited))) == Inf

    %% get the current node
    [~, current_node] = min(distances./unvisited);
    
    %% get all unvisited nodes connected to current node
	[row, col] = find(Graph(:,1:2) == current_node & fliplr(ismember(Graph(:,1:2),nodes(logical(unvisited)))));
    col = mod(col,2)+1;
    idx = sub2ind(size(Graph),row, col);
    children = nodes(ismember(nodes,Graph(idx)) & unvisited);
    
    %% update distances
    if ~isempty(children)
        tentative = distances(current_node)+Graph(row,3)' < distances(children);
        distances(children(tentative)) = distances(current_node)+Graph(row(tentative),3);
        reference(ismember(nodes,children(tentative))) = current_node;
    end
    
    %% remove current node from graph
    unvisited(nodes == current_node) = 0;
end
distances
reference
if cost == Inf
    path = []';
end
end
