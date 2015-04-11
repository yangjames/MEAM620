function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
figure(1)
clf
[x,y,z] = get_rect_vals(map.boundary);
h(1) = patch(x,y,z,'w','FaceAlpha',0.0);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
%xlim(map.boundary([1,4]))
%ylim(map.boundary([2,5]))
%zlim(map.boundary([3,6]))
grid on
for i = 2:size(map.blocks,1)+1
    [x,y,z] = get_rect_vals(map.blocks(i-1,:));
    h(i) = patch(x,y,z,map.blocks(i-1,7:9)/255);
end
if ~isempty(path)
    h(end+1) = plot3(path(:,1),path(:,2),path(:,3),'k-');
end
end