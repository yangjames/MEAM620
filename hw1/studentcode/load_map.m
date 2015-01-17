function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

%% parse text file for boundary and obstacles
% read in text file
text = fileread(filename);

% split text by line
split_texts = strsplit(text,'\n');

% find uncommented and otherwise valid phrases
valid_boundary = regexp(split_texts,'^\s*boundary(\s*\d+\.?\d*){6}\s*$');
valid_blocks = regexp(split_texts,'^\s*block(\s*\d+\.?\d*){9}\s*$');

% get indices of valid phrases
blocks_idx = find(~cellfun(@isempty, valid_blocks));

% extract boundary and obstacle values
boundary_text = split_texts{~cellfun(@isempty,valid_boundary)};
boundary = cell2mat(textscan(boundary_text(length('boundary')+1:end), '%f'))
blocks = zeros(9,length(blocks_idx));
for i = 1:length(blocks_idx)
    block_text = split_texts{blocks_idx(i)};
    blocks(:,i) = cell2mat(textscan(block_text(length('block')+1:end), '%f'));
end
blocks
%% create map
map = zeros(0, 0);
end
