function h = plot_map(boundary, blocks)
[x,y,z] = get_rect_vals(boundary);
h(1) = patch(x,y,z,'w','FaceAlpha',0.0);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
grid on
for i = 2:size(blocks,1)
    [x,y,z] = get_rect_vals(blocks(i,:));
    h(i) = patch(x,y,z,blocks(i,7:9)/255);
end