function coordinate = node_to_xyz(map,node)
% returns an nx3 matrix of xyz coordinates corresponding to node value

num_x = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_y = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_plane = num_x*num_y;

plane_proj = mod(node-1,num_plane)+1;
coordinate = [mod(plane_proj-1,num_x)*map.xy_res + map.boundary(1) ...
                floor((plane_proj-1)/num_x)*map.xy_res + map.boundary(2) ...
                floor((node-1)/num_plane)*map.z_res + map.boundary(3)];