clear all
load sample_path.mat

%{d
max_speed = 2;
c_t = 0;
t_idx = 1;

% find indices where path switches directions
pivot_idx = find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;

% calculate distance between pivot points
distances = sqrt(sum((path0([1; pivot_idx],:)-path0([pivot_idx;size(path0,1)],:)).^2,2));

% gather time stamps between pivot points
t = distances/max_speed;


%}

%{
max_speed = 2;
c_t = 0;
t_idx = 1;

% find indices where path switches directions
pivot_idx = find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;

% calculate distance between pivot points
distances = sqrt(sum((path0([1; pivot_idx],:)-path0([pivot_idx;size(path0,1)],:)).^2,2));

% gather time stamps between pivot points
t = distances/max_speed;
a_x = zeros(length(t),4);
a_y = a_x;
a_z = a_x;

a_x(1,:) = [path0(1,1) 0 0 path0(2,1)/t(1)^3];
a_y(1,:) = [path0(1,2) 0 0 path0(2,2)/t(1)^3];
a_z(1,:) = [path0(1,2) 0 0 path0(2,3)/t(1)^3];

for i = 1:length(t)-1
    a_x(i+1,:) = [path0(i+1,1) ...
                a_x(i,2)+2*a_x(i,3)*t(i)+3*a_x(i,4)*t(i)^2 ...
                a_x(i,3)+3*a_x(i,4)*t(i) ...
                1/t(i)^3*(path0(i+1,1)-a_x(i,1)-a_x(i,2)*t(i)-a_x(i,3)*t(i)^2)];
    a_y(i+1,:) = [path0(i+1,2) ...
                a_y(i,2)+2*a_y(i,3)*t(i)+3*a_y(i,4)*t(i)^2 ...
                a_y(i,3)+3*a_y(i,4)*t(i) ...
                1/t(i)^3*(path0(i+1,2)-a_y(i,1)-a_y(i,2)*t(i)-a_y(i,3)*t(i)^2)];
    a_z(i+1,:) = [path0(i+1,3) ...
                a_z(i,2)+2*a_z(i,3)*t(i)+3*a_z(i,4)*t(i)^2 ...
                a_z(i,3)+3*a_z(i,4)*t(i) ...
                1/t(i)^3*(path0(i+1,3)-a_x(i,1)-a_z(i,2)*t(i)-a_z(i,3)*t(i)^2)];
end

time = 0:0.01:30;
c_t = c_t+t(t_idx);
total_time = sum(t);
for i = 1:length(time)
    if time(i) < total_time
        if time(i) >= c_t
            t_idx = t_idx+1;
            c_t = c_t+t(t_idx);
        end
        x(i) = a_x(t_idx,:)*[1 time(i) time(i)^2 time(i)^3]';
        y(i) = a_y(t_idx,:)*[1 time(i) time(i)^2 time(i)^3]';
        z(i) = a_z(t_idx,:)*[1 time(i) time(i)^2 time(i)^3]';
    else
        x(i) = path0(end,1);
        y(i) = path0(end,2);
        z(i) = path0(end,3);
    end
end
figure(6)
clf
plot3(x,y,z,'b.')
%}
%{
max_speed = 0.5;

% find indices where path switches directions
pivot_idx = find(sqrt(sum(diff(diff(path0)).^2,2))>100*eps)+1;

% calculate distance between pivot points
distances = sqrt(sum((path0([1; pivot_idx],:)-path0([pivot_idx;size(path0,1)],:)).^2,2));

% gather time stamps between pivot points
dt_stamps = distances/max_speed;

c_time = dt_stamps(1);
t_idx = 1;
t_total = sum(dt_stamps);

time = 0:0.01:30
for i = 1:length(time)
    t = time(i)
    if t <= t_total
        if t >= c_time
            t_idx = t_idx+1;
            c_time = c_time + dt_stamps(t_idx);
        end
        t = t-sum(dt_stamps(1:t_idx-1));
        X_0 = [path0(pivot_idx(t_idx),:);...
            0 0 0;...
            0 0 0;...
            path0(pivot_idx(t_idx+1),:);...
            0 0 0;...
            0 0 0];
        a = get_interp_weights(X_0,dt_stamps(t_idx));
        X = [1 t t^2 t^3 t^4 t^5;...
            0 1 2*t 3*t^2 4*t^3 5*t^4;...
            0 0 2 6*t 12*t^2 20*t^3]*a;
        pos = X(1,:)';
        vel = X(2,:)';
        acc = X(3,:)';
    else
        pos = path0(end,:)';
        vel = [0 0 0]';
        acc = [0 0 0]';
    end
end
%}