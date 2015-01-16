%% min heap test
num_edges = 10;
%{
stats = [    0.7060    1.0000;...
    0.0318    2.0000;...
    0.2769    3.0000;...
    0.0462    4.0000;...
    0.0971    5.0000;...
    0.8235    6.0000;...
    0.6948    7.0000;...
    0.3171    8.0000;...
    0.9502    9.0000;...
    0.0344   10.0000];
%}
stats = [0.0344 0.0462 0.2769 0.3171 0.0971 0.823 0.6948 0.7060 0.9502 0.0318]';
%{
for i = 2:num_edges
    a(i,:) = stats(i,:);
    iterator = i;
    while iterator > 1
        idx = floor(iterator/2);
        if a(idx,1) > a(iterator,1)
            temp = a(idx,:);
            a(idx,:) = a(iterator,:);
            a(iterator,:) = temp;
            iterator = idx;
        else
            break;
        end
    end
end
%}
stats = add_node(stats,10)
stats(1) = stats(end);
stats = delete_node(stats,1,9)