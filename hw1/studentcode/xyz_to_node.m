function node = xyz_to_node(map,coordinate)
% returns a column vector of nodes mapped to a coordinate

num_rows = ceil(map.boundary(4)/map.xy_res);
num_cols = ceil(map.boundary(5)/map.xy_res);
num_plane = num_rows*num_cols;

z_comp = ceil(coordinate(:,3)/map.z_res)*num_plane;
y_comp = ceil(coordinate(:,2)/map.xy_res)*num_rows;
x_comp = ceil(coordinate(:,1)/map.xy_res);

node = x_comp + y_comp + z_comp;