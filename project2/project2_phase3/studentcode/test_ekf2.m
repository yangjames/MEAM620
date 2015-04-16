%% load data
clear all;
close all;
load data/studentdata9;

init_script;
 
%% interleave time vectors
N = length(data);
all_time = zeros(1,N);
for i = 1:N
    all_time(i) = data(i).t;
end

%% run filter
X = [];
Z = [];
idx = 1;
for i=1:N
    [X_out, Z_out] = ekf2_handle(data(i), params);
    if ~isempty(X_out)
        X(1:15,idx) = X_out;
        Z(1:6,idx) = Z_out;
        idx = idx+1;
    end
end

figure, grid on, hold on
subplot(2,3,1)
plot(linspace(0,all_time(end),idx-1),X(1,:),'r-',time,vicon(1,:),'b-');
title('x')
subplot(2,3,2)
plot(linspace(0,all_time(end),idx-1),X(2,:),'r-',time,vicon(2,:),'b-');
title('y')
subplot(2,3,3)
plot(linspace(0,all_time(end),idx-1),X(3,:),'r-',time,vicon(3,:),'b-');
title('z')
subplot(2,3,4)
plot(linspace(0,all_time(end),idx-1),X(4,:),'r-',time,vicon(4,:),'b-');
title('\phi')
subplot(2,3,5)
plot(linspace(0,all_time(end),idx-1),X(5,:),'r-',time,vicon(5,:),'b-');
title('\theta')
subplot(2,3,6)
plot(linspace(0,all_time(end),idx-1),X(6,:),'r-',time,vicon(6,:),'b-');
title('\psi')