function ob=out_bounds(map,points)
%% get true bounds
n_bound = map.boundary+[-map.xy_res -map.xy_res -map.z_res map.xy_res map.xy_res map.z_res]*1/100;% + margin;

%% get points that are out of bounds
ob = sum(bsxfun(@gt, n_bound(1:3), points) | bsxfun(@lt, n_bound(4:6), points),2)>0;
