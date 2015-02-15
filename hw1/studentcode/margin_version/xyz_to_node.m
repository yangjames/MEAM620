function node = xyz_to_node(map,coordinate)
% returns a column vector of nodes mapped to a coordinate

num_x = floor((map.boundary(4)-map.boundary(1))/map.xy_res)+1;
num_y = floor((map.boundary(5)-map.boundary(2))/map.xy_res)+1;
num_depth = floor((map.boundary(6)-map.boundary(3))/map.z_res)+1;
num_plane = num_x*num_y;

z_comp = round((coordinate(:,3)-map.boundary(3))/map.z_res)*num_plane;
if z_comp>=num_plane*num_depth
    z_comp = 0;
end
y_comp = round((coordinate(:,2)-map.boundary(2))/map.xy_res)*num_x;
x_comp = round((coordinate(:,1)-map.boundary(1))/map.xy_res)+1;

node = x_comp + y_comp + z_comp;
