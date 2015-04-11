clear all
theta = 0:0.01:2*pi;
theta_new = zeros(size(theta));
for i = 1:length(theta)
    t = (i-1)/length(theta)*10;
    a = get_interp_weights([0;0;0;2*pi;0;0],10);
    X = [1 t t^2 t^3 t^4 t^5;...
        0 1 2*t 3*t^2 4*t^3 5*t^4;...
        0 0 2 6*t 12*t^2 20*t^3]*a;
    theta_new(i) = X(1);
end
figure(8)
clf
plot(linspace(0,10,length(theta)),theta_new,'r.')