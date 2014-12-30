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
if nargin ~= 3
    error('Incorrect number of arguments');
end

path = zeros(0,1);
cost = Inf;

%% special case with an empty graph
if isempty(Graph)
    return;
end

nodes = unique(Graph(:,1:2));

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

%% initialize graph properties
graph = {};
for i = 1:length(nodes)
    graph{i}.node = nodes(i);
    
    [row, col] = find(Graph(:,1:2) == nodes(i));
    col = mod(col,2)+1;
    idx_mat = sortrows([row col],1);
    row = idx_mat(:,1);
    col = idx_mat(:,2);
    idx = sub2ind(size(Graph),row, col);
    for j = 1:length(idx)
        children(j) = nodes(nodes == Graph(idx(j)));
    end
    [graph{i}.children, uidx] = unique(children);
    graph{i}.distances = Graph(row,3);
    graph{i}.distances = graph{i}.distances(uidx);
    
    row = [];
    col = [];
    idx_mat = [];
    idx = [];
    children = [];
end

%for i = 1:length(nodes)
    
%end

distance = Inf(size(nodes));
previous = NaN(size(nodes));
unvisited = true(size(nodes));
distance(nodes == start) = 0;

%% loop until we are at our destination or we can't get to our goal
while unvisited(nodes == goal) && min(distance(unvisited)) ~= Inf
    % get unvisited node with smallest distance
    [dist, curr_idx] = min(distance./unvisited);
    current_node = graph{curr_idx}.node;
    
    % obtain tentative distances and update distances if necessary
    tentative = dist + graph{curr_idx}.distances < distance(ismember(nodes, graph{curr_idx}.children));
    distance(ismember(nodes, graph{curr_idx}.children(tentative))) = dist+graph{curr_idx}.distances(tentative);
    previous(ismember(nodes,graph{curr_idx}.children(tentative))) = current_node;

    % remove current node from graph
    unvisited(curr_idx) = false;
end
cost = distance(nodes == goal);

%% find the path if it exists
if cost ~= Inf
    path = NaN(size(nodes));
    iterator = length(path);
     path_node = goal;
     while path_node ~= start
         path(iterator) = path_node;
         iterator = iterator - 1;
         path_node = previous(nodes == path_node);
     end
     path(iterator) = start;
     if iterator < length(path)
        path(1:iterator-1) = [];
     end
end