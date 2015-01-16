function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
text = fileread(filename);
C = strsplit(text,'\n')
boundary = regexp(C,'^\s*boundary(\s*\d+\.?\d*){6}')
blocks = regexp(C,'^\s*block(\s*\d+\.?\d*){9}')

map = zeros(0, 0);
end
