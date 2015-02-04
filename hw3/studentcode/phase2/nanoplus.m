function params = nanoplus()
% NANOPLUS basic parameters for nanoplus quadrotor
%   nanoplus outputs the basic parameters (mass, gravity and inertia) of
%   the quadrotor nanoplus

m = 0.176; %kg: nanoplus without gumstix and camera
g = 9.81;
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = 0.086;

% Ixx = I(1,1);
% Iyy = I(2,2);
% Izz = I(3,3);

params.maxangle = 85*pi/180; %you can specify the maximum commanded angle here
params.maxF = 2.5*m*g;
params.minF = 0.05*m*g;

% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m

% orientation gains
%params.Kp_o = 60; params.Kd_o = 3.4; params.Ki_o = 0.0;
params.Kp_o = 7; params.Kd_o = 1; params.Ki_o = 0.0;

% thrust gains
%params.Kp_t = 125; params.Kd_t = 1.25; params.Ki_t = 0.0;
params.Kp_t = 7; params.Kd_t = 0.5; params.Ki_t = 0.0;

% moment gains
%params.Kp_m = 20; params.Kd_m = 0.5;
params.Kp_m = 30; params.Kd_m = 0.17;

params.Kp_m_y = 4; params.Kd_m_y = 0.5;
end
