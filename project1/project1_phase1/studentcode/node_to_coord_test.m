map = load_map('sample_maps/emptyMap.txt',0.5, 0.5, 0.1)
start = [1.81 2.40, 5.5];
stop = [9.8 8.7 0.2];

num_x = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_y = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_depth = floor((map.boundary(6)-map.boundary(3))/map.z_res)+1;
num_plane = num_x*num_y;
num_nodes = num_depth*num_plane


collide(map,[start;stop])
plot_path(map,[start;stop])
[path,nodes_expanded]=dijkstrav2(map,start,stop,false)

plot_path(map,path)
%{
neighbors_26 = [repmat([-map.xy_res; 0; map.xy_res],9,1)...
    repmat([repmat(-map.xy_res,3,1); zeros(3,1); repmat(map.xy_res,3,1)],3,1)...
    [repmat(-map.z_res,9,1); zeros(9,1); repmat(map.z_res,9,1)]];
neighbors_dist = sqrt(sum(neighbors_26.^2,2));
neighbors_26(14,:) = [];
neighbors_dist(14) = [];


current_coord = [1 5 3.9]
collide(map,current_coord)
neighbors_coord = bsxfun(@plus,neighbors_26,current_coord)
c = collide(map,neighbors_coord)
%}
%{
num_x = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_y = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_depth = floor((map.boundary(6)-map.boundary(3))/map.z_res)+1;
num_plane = num_x*num_y;
num_nodes = num_depth*num_plane

nodes = (1:num_nodes)';
coord = node_to_xyz(map,nodes);

test_nodes = xyz_to_node(map,coord);

sum(nodes == test_nodes) == num_nodes

xyz_to_node(map,[10 19.7 6])
%}