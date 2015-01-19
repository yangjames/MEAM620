function node = xyz_to_node(map,coordinate)
% returns a column vector of nodes mapped to a coordinate

num_rows = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_cols = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_plane = num_rows*num_cols;

z_comp = floor((coordinate(:,3)-map.boundary(3))/map.z_res)*num_plane;
y_comp = floor((coordinate(:,2)-map.boundary(2))/map.xy_res)*num_rows;
x_comp = floor((coordinate(:,1)-map.boundary(1))/map.xy_res);

node = x_comp + y_comp + z_comp;