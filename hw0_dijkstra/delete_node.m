function heap = delete_node(heap,iterator,len)
%iterator = 1;
%while iterator <= len/2
    left = 2*iterator;
    right = 2*iterator+1;
   	smallest = iterator;
    if left <= len && heap(left,1) < heap(iterator,1)
        smallest = left;
    end
    if right <= len && heap(right,1) < heap(iterator,1)
        smallest = right;
    end
    if smallest ~= iterator
        temp = heap(smallest,:);
        heap(smallest,:) = heap(iterator,:);
        heap(iterator,:) = temp;
        heap = delete_node(heap,smallest,len);
    end
%end