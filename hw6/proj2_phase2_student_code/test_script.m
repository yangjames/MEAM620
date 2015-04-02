clear all
close all

%% load data
dataset = 9; % 1, 4, or 9
load(['data/studentdata' num2str(dataset) '.mat']);

%% process data
global feature_plot
figure(1)
clf
img = imshow(data(1).img);
hold on
feature_plot = plot(0,0,'g*');

figure(2)
vicon_path_plot = plot3(0,0,0,'bo');
axis equal
grid on
hold on
scale = 0.1;
x_vicon_att = plot3(0,0,0,'r-');
y_vicon_att = plot3(0,0,0,'g-');
z_vicon_att = plot3(0,0,0,'b-');

cam_path_plot = plot3(0,0,0,'ro');
x_cam_att = plot3(0,0,0,'m-');
y_cam_att = plot3(0,0,0,'c-');
z_cam_att = plot3(0,0,0,'k-');

xlim([min(vicon(1,:))-scale max(vicon(1,:))+scale])
ylim([min(vicon(2,:))-scale max(vicon(2,:))+scale])
zlim([min(vicon(3,:))-scale max(vicon(3,:))+scale])
xlabel('x')
ylabel('y')
zlabel('z')

cam_vel = zeros(3,length(data));
cam_eul = zeros(3,length(data));

tic
t_start = toc;
num_bytes = 0;
for i = 1:length(data)
    % estimate pose
    [vel,eul] = estimate_vel(data(i));
    if ~isempty(vel) && ~isempty(eul)
        cam_vel(:,i) = vel;
        cam_eul(:,i) = eul;
    end
    
    % redraw plots
    [~,vicon_idx] = min(abs(time-data(i).t));
    if i > 1 && mod(i,3) == 0
        set(img,'cdata',data(i).img)
        set(vicon_path_plot,'xdata',vicon(1,1:vicon_idx),...
            'ydata',vicon(2,1:vicon_idx),...
            'zdata',vicon(3,1:vicon_idx))

        vicon_pos = vicon(1:3,vicon_idx);
        R_vicon = get_rot(vicon(4,vicon_idx),vicon(5,vicon_idx),vicon(6,vicon_idx),'ZXY');
        set(x_vicon_att,'xdata',[0 R_vicon(1,1)]*scale + vicon_pos(1),'ydata',[0 R_vicon(2,1)]*scale + vicon_pos(2),'zdata',[0 R_vicon(3,1)]*scale + vicon_pos(3))
        set(y_vicon_att,'xdata',[0 R_vicon(1,2)]*scale + vicon_pos(1),'ydata',[0 R_vicon(2,2)]*scale + vicon_pos(2),'zdata',[0 R_vicon(3,2)]*scale + vicon_pos(3))
        set(z_vicon_att,'xdata',[0 R_vicon(1,3)]*scale + vicon_pos(1),'ydata',[0 R_vicon(2,3)]*scale + vicon_pos(2),'zdata',[0 R_vicon(3,3)]*scale + vicon_pos(3))

        R_cam = get_rot(cam_eul(1,i),cam_eul(2,i),cam_eul(3,i),'ZXY');
        set(x_cam_att,'xdata',[0 R_cam(1,1)]*scale + cam_vel(1,i),'ydata',[0 R_cam(2,1)]*scale + cam_vel(2,i),'zdata',[0 R_cam(3,1)]*scale + cam_vel(3,i))
        set(y_cam_att,'xdata',[0 R_cam(1,2)]*scale + cam_vel(1,i),'ydata',[0 R_cam(2,2)]*scale + cam_vel(2,i),'zdata',[0 R_cam(3,2)]*scale + cam_vel(3,i))
        set(z_cam_att,'xdata',[0 R_cam(1,3)]*scale + cam_vel(1,i),'ydata',[0 R_cam(2,3)]*scale + cam_vel(2,i),'zdata',[0 R_cam(3,3)]*scale + cam_vel(3,i))

        fprintf(repmat('\b',1,num_bytes));
        num_bytes = fprintf('time: %6.6f',data(i).t);
        drawnow
        t_end = toc;
        if t_end-t_start < data(i).t
            pause(data(i).t-t_end)
        end
    end
end
data_ts = [];
for i = 1:length(data)
    data_ts(i) = data(i).t;
end

figure(3)
clf
plot(data_ts,cam_vel(1,:),'r-')
hold on
plot(time,vicon(7,:),'b-')
grid on
xlabel('time')
ylabel('v_x')
legend('calculated','vicon')

figure(4)
clf
plot(data_ts,cam_vel(2,:),'r-')
hold on
plot(time,vicon(8,:),'b-')
grid on
xlabel('time')
ylabel('v_y')
legend('calculated','vicon')

figure(5)
clf
plot(data_ts,cam_vel(3,:),'r-')
hold on
plot(time,vicon(9,:),'b-')
grid on
xlabel('time')
zlabel('v_z')
legend('calculated','vicon')

fprintf('\n');