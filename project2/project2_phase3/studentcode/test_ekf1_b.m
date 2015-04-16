%% load data
clear all;
close all;
load data/studentdata1;

init_script;

 
%% interleave time vectors
 
all_time = [data.t time; zeros(1,length(data)) ones(1,length(time)); 1:length(data) 1:length(time)];
all_time = all_time(:,nth(2,2, @sort,all_time(1,:)));
N = length(all_time);
 
%% run filter
X = zeros(9,N);
Z = zeros(6,N);
for i=1:N
    if all_time(2,i) == 1
        vic = struct('t', all_time(1,i), 'vel', vicon(7:12,all_time(3,i)));
        [X(:,i), Z(:,i)] = ekf1b_handle([], vic, params);
    else
        sensor = data(all_time(3,i));
        [X(:,i), Z(:,i)] = ekf1b_handle(sensor, [], params);
    end
end

figure, grid on, hold on
subplot(2,3,1)
plot(linspace(0,time(end),N),X(1,:),'r-',time,vicon(1,:),'b-');
title('x')
subplot(2,3,2)
plot(linspace(0,time(end),N),X(2,:),'r-',time,vicon(2,:),'b-');
title('y')
subplot(2,3,3)
plot(linspace(0,time(end),N),X(3,:),'r-',time,vicon(3,:),'b-');
title('z')
subplot(2,3,4)
plot(linspace(0,time(end),N),X(4,:),'r-',time,vicon(4,:),'b-');
title('\phi')
subplot(2,3,5)
plot(linspace(0,time(end),N),X(5,:),'r-',time,vicon(5,:),'b-');
title('\theta')
subplot(2,3,6)
plot(linspace(0,time(end),N),X(6,:),'r-',time,vicon(6,:),'b-');
title('\psi')