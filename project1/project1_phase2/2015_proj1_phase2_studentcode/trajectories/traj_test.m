t = 0:0.1:12;
x = zeros(size(t));
y = x;
z = x;
vx =x;
vy = x;
vz = x;

for i = 1:length(t)
    [desired_state] = diamond(t(i), 1);
    x(i) = desired_state.pos(1);
    y(i) = desired_state.pos(2);
    z(i) = desired_state.pos(3);
    vx(i) = desired_state.vel(1);
    vy(i) = desired_state.vel(2);
    vz(i) = desired_state.vel(3);
end

figure(1)
clf
plot3(x,y,z,'b-')
grid on

figure(2)
clf
quiver3(x,y,z,vx,vy,vz)
grid on