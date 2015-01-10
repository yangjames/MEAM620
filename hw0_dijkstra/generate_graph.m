function G = generate_graph(num_nodes, num_edges)
G = zeros(num_edges,3);
G(:,1:2) = ceil(rand(num_edges,2)*num_nodes);
G(:,3) = rand(num_edges,1);
indicator = G(:,1) == G(:,2);
while sum(indicator) > 0
    G(indicator,1:2) = ceil(rand(sum(indicator),2)*num_nodes);
    indicator = G(:,1) == G(:,2);
end
