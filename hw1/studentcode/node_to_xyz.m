function coordinate = node_to_xyz(map,node)
% returns an nx3 matrix of xyz coordinates corresponding to node value

num_rows = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_cols = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_plane = num_rows*num_cols;


coordinate = [(mod(mod(node-1,num_plane),num_cols))*map.xy_res + map.boundary(1) ...
                floor(mod(node-1,num_plane)/num_rows)*map.xy_res + map.boundary(2) ...
                floor((node-1)/num_plane)*map.z_res + map.boundary(3)];