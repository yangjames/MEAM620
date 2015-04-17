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
timing = [];
tic
for i=1:N
    t_start = toc;
    [X_out, Z_out] = ekf2_handle(data(i), params);
    t_end = toc;
    if ~isempty(X_out)
        X(1:15,idx) = X_out;
        Z(1:9,idx) = Z_out;
        timing(idx) = t_end-t_start;
        idx = idx+1;
    end
end

figure, grid on, hold on
m_time = linspace(0,all_time(end),idx-1);
subplot(3,3,1)
plot(m_time,X(1,:),'r-',time,vicon(1,:),'b-',m_time,Z(1,:),'g--');
title('x')
subplot(3,3,2)
plot(m_time,X(2,:),'r-',time,vicon(2,:),'b-',m_time,Z(2,:),'g--');
title('y')
subplot(3,3,3)
plot(m_time,X(3,:),'r-',time,vicon(3,:),'b-',m_time,Z(3,:),'g--');
title('z')

subplot(3,3,4)
plot(m_time,X(4,:),'r-',time,vicon(7,:),'b-',m_time,Z(4,:),'g--');
title('$\dot{x}$','Interpreter','LaTeX')
subplot(3,3,5)
plot(m_time,X(5,:),'r-',time,vicon(8,:),'b-',m_time,Z(5,:),'g--');
title('$\dot{y}$','Interpreter','LaTeX')
subplot(3,3,6)
plot(m_time,X(6,:),'r-',time,vicon(9,:),'b-',m_time,Z(6,:),'g--');
title('$\dot{z}$','Interpreter','LaTeX')

subplot(3,3,7)
plot(m_time,X(7,:),'r-',time,vicon(4,:),'b-',m_time,Z(7,:),'g--');
title('\phi')
subplot(3,3,8)
plot(m_time,X(8,:),'r-',time,vicon(5,:),'b-',m_time,Z(8,:),'g--');
title('\theta')
subplot(3,3,9)
plot(m_time,X(9,:),'r-',time,vicon(6,:),'b-',m_time,Z(9,:),'g--');
title('\psi')

figure, grid on
plot(1:idx-1,timing,'b--')
ylabel('dt')
xlabel('index')