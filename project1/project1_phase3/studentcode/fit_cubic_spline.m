function coeff = fit_cubic_spline(truncated_path, dt_stamps)
% calculate distance between pivot points
distances = sqrt(sum((truncated_path(1:end-1,:)-truncated_path(2:end,:)).^2,2));
% gather time stamps between pivot points
%dt_stamps = cumsum([0;sqrt(distances)*0.7]);

A = zeros(length(distances)*4);
X = zeros(length(distances)*4,3);

% add position constraints
for i = 1:length(distances)
    A((i-1)*2+1:(i-1)*2+2,(i-1)*4+1:(i-1)*4+4) = [1 dt_stamps(i) dt_stamps(i)^2 dt_stamps(i)^3;...
                            1 dt_stamps(i+1) dt_stamps(i+1)^2 dt_stamps(i+1)^3];
    X((i-1)*2+1:(i-1)*2+2,:) = [truncated_path(i,:);truncated_path(i+1,:)];
end

% add end point velocity constraints
A(length(distances)*2+1,1:4) = [0 1 2*dt_stamps(1) 3*dt_stamps(1)^2];
A(length(distances)*2+2,end-3:end) = [0 1 2*dt_stamps(end) 3*dt_stamps(end)^2];

% add velocity and acceleration constraints
for i = 1:length(distances)-1
    A(length(distances)*2+2+i,(i-1)*4+1:(i-1)*4+8)=...
        [0 1 2*dt_stamps(i+1) 3*dt_stamps(i+1)^2 0 -1 -2*dt_stamps(i+1) -3*dt_stamps(i+1)^2];
    A(length(distances)*2+2+i+length(distances)-1,(i-1)*4+1:(i-1)*4+8) = ...
        [0 0 2 6*dt_stamps(i+1) 0 0 -2 -6*dt_stamps(i+1)];
end

size(truncated_path)
size(distances)
size(A)
size(X)
coeff=A\X;