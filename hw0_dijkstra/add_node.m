function heap = add_node(heap, len)
iterator = len;
while iterator ~= 1
    idx = floor(iterator/2);
    if heap(idx,1) > heap(iterator,1)
        temp = heap(idx,:);
        heap(idx,:) = heap(iterator,:);
        heap(iterator,:) = temp;
        iterator = idx;
    else
        break;
    end
end
%{
for i = 2:len
    iterator = i;
    while iterator > 1
        idx = floor(iterator/2);
        if heap(idx,1) > heap(iterator,1)
            temp = heap(idx,:);
            heap(idx,:) = heap(iterator,:);
            heap(iterator,:) = temp;
            iterator = idx;
        else
            break;
        end
    end
end
%}