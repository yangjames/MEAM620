% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) ekf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) ekf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function



%% create ekf1 no bias function handles
p = sym('x',[3 1]);
q = sym('a',[3 1]);

v_m=sym('v',[3 1]);
w_m=sym('w',[3 1]);

n_v = sym('nv',[3 1]);
n_g = sym('ng',[3 1]);

assume([p q v_m w_m],'real')


x = [p; q];

G = [cos(q(2)) 0 -cos(q(1))*sin(q(2));
    0 1 sin(q(1));
    sin(q(2)) 0 cos(q(1))*cos(q(2))];

omg = G^-1*(w_m-n_g);
x_dot = [v_m-n_v;
    omg];

params.A1=matlabFunction(simplify(jacobian(x_dot,x)));
params.U1=matlabFunction(simplify(jacobian(x_dot,[n_v;n_g])));
params.f1=matlabFunction(x_dot);

%% create ekf1 with bias function handles
p = sym('x',[3 1]);
q = sym('a',[3 1]);

v_m=sym('v',[3 1]);
w_m=sym('w',[3 1]);
b_g=sym('b_g',[3 1]);

n_v = sym('n_v',[3 1]);
n_g = sym('n_g',[3 1]);
n_bg = sym('n_bg',[3 1]);

assume([p q v_m w_m],'real')


x = [p; q; b_g];

G = [cos(q(2)) 0 -cos(q(1))*sin(q(2));
    0 1 sin(q(1));
    sin(q(2)) 0 cos(q(1))*cos(q(2))];


x_dot = simplify([v_m-n_v;
        G^-1*(w_m-b_g-n_g);
        n_bg]);

params.A1_b=matlabFunction(simplify(jacobian(x_dot,x)));
params.U1_b=matlabFunction(simplify(jacobian(x_dot,[n_v;n_g;n_bg])));
params.f1_b=matlabFunction(simplify(x_dot));

%% create ekf2 function handles

p = sym('x',[3 1]);
q = sym('a',[3 1]);
p_dot = sym('v',[3 1]);
b_g = sym('b_g',[3 1]);
b_a = sym('b_a',[3 1]);

x = [p;q;p_dot;b_g;b_a];

a_m = sym('a_m',[3 1]);
w_m = sym('w_m',[3 1]);
n_g = sym('n_g',[3 1]);
n_a = sym('n_a',[3 1]);
n_bg = sym('n_bg',[3 1]);
n_ba = sym('n_ba',[3 1]);

assume([p q p_dot b_g b_a n_g n_a n_bg n_ba w_m],'real')

p_ddot = sym('a',[3 1]);
g = [0 0 -9.81]';
%R = get_rot(q(1),q(2),q(3),'zxy');
R = [cos(q(3))*cos(q(2))-sin(q(1))*sin(q(3))*sin(q(2)) -cos(q(1))*sin(q(3)) cos(q(3))*sin(q(2))+cos(q(2))*sin(q(1))*sin(q(3));...
    cos(q(2))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2)) cos(q(1))*cos(q(3)) sin(q(3))*sin(q(2))-cos(q(3))*cos(q(2))*sin(q(1));...
    -cos(q(1))*sin(q(2)) sin(q(1)) cos(q(1))*cos(q(2))];

G = [cos(q(2)) 0 -cos(q(1))*sin(q(2));
    0 1 sin(q(1));
    sin(q(2)) 0 cos(q(1))*cos(q(2))];

x_dot = simplify([p_dot; G\(w_m-b_g-n_g); g+R*(a_m-b_a-n_a);n_bg;n_ba]);

z = [p;q;p_dot];

params.A2=matlabFunction(simplify(jacobian(x_dot,x)));
params.U2=matlabFunction(simplify(jacobian(x_dot,[n_a;n_g;n_bg;n_ba])));
params.f2=matlabFunction(x_dot);
params.C2=matlabFunction(simplify(jacobian(z,x)));

%% create ekf2 no bias function handles

p = sym('x',[3 1]);
q = sym('a',[3 1]);
p_dot = sym('v',[3 1]);

x = [p;q;p_dot];

a_m = sym('a_m',[3 1]);
w_m = sym('w_m',[3 1]);
n_g = sym('n_g',[3 1]);
n_a = sym('n_a',[3 1]);

assume([p q p_dot n_g n_a w_m],'real')

p_ddot = sym('a',[3 1]);
g = [0 0 -9.81]';
R = get_rot(q(1),q(2),q(3),'zxy');
%{
R = [cos(q(3))*cos(q(2))-sin(q(1))*sin(q(3))*sin(q(2)) -cos(q(1))*sin(q(3)) cos(q(3))*sin(q(2))+cos(q(2))*sin(q(1))*sin(q(3));...
    cos(q(2))*sin(q(3))+cos(q(3))*sin(q(1))*sin(q(2)) cos(q(1))*cos(q(3)) sin(q(3))*sin(q(2))-cos(q(3))*cos(q(2))*sin(q(1));...
    -cos(q(1))*sin(q(2)) sin(q(1)) cos(q(1))*cos(q(2))];
%}
G = [cos(q(2)) 0 -cos(q(1))*sin(q(2));
    0 1 sin(q(1));
    sin(q(2)) 0 cos(q(1))*cos(q(2))];

x_dot = simplify([p_dot; G\(w_m-n_g); g+R*(a_m-n_a)]);

z = [p;q;p_dot];

params.A2_nb=matlabFunction(simplify(jacobian(x_dot,x)));
params.U2_nb=matlabFunction(simplify(jacobian(x_dot,[n_g;n_a])));
params.f2_nb=matlabFunction(x_dot);
params.C2_nb=matlabFunction(simplify(jacobian(z,x)));


ekf1_handle = @(sensor, vic) ekf1(sensor, vic, params);
ekf2_handle = @(sensor) ekf2_nb(sensor,params);
ekf1b_handle = @(sensor,vic) ekf1_b(sensor,vic,params);