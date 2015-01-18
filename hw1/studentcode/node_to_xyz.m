function coordinate = node_to_xyz(map,node)
% returns an nx3 matrix of xyz coordinates corresponding to node value

num_rows = floor(map.boundary(4)/map.xy_res);
num_cols = floor(map.boundary(5)/map.xy_res);
num_plane = num_rows*num_cols;

mod(node,num_plane)
coordinate = [mod(mod(node,num_plane),num_cols)*map.xy_res ...
                floor(mod(node,num_plane)/num_rows)*map.xy_res ...
                round(node/num_plane)*map.z_res];