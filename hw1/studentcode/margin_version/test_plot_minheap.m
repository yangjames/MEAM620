function test_plot_minheap(Q)

num_nodes = length(Q);

figure(9)
clf
hold on

for ii = 1:num_nodes
    y = floor(log2(ii));
    x = (ii-2^y)/2^(y-1) - 1 + 2^(-y);
    text(x,y,num2str(Q(ii)))
end
    
hold off

set(gca,'xlim',[-1,1],'ylim',[-1,floor(log2(ii))+1],'ydir','reverse')
    drawnow