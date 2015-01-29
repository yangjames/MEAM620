function plot_map(map, test_points)
boundary = map.boundary;
blocks = map.blocks;
margin = map.margin;
figure(1)
clf
h = zeros(2+size(blocks,1),1);
[x,y,z] = get_rect_vals(boundary);
h(1) = patch(x,y,z,'w','FaceAlpha',0.0);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
xlim(boundary([1,4]))
ylim(boundary([2,5]))
zlim(boundary([3,6]))
grid on
for i = 2:size(blocks,1)
    [x,y,z] = get_rect_vals(blocks(i,:));
    h(i) = patch(x,y,z,blocks(i,7:9)/255);
end
[x,y,z] = sphere(10);
for i = size(blocks,1)+1:size(test_points,1)+size(blocks,1)
    idx = i - size(blocks,1);
    h(i) = surf(x*margin+test_points(idx,1),...
        y*margin+test_points(idx,2),...
        z*margin+test_points(idx,3));
end
