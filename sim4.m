%% Load some initial values:

% Conversion factors:
deg2rad = pi / 180;
rad2deg = 180 / pi;
in2ft = 1 / 12;
ft2in = 12;
mph2ftps = 5280 / 3600;
ftps2mph = 3600 / 5280;

% Bicycle model parameters:
W = 3000; % lbs
Ws = 2700; % lbs
x1 = 3.5; % ft
x2 = -4.5; % ft
h = -1.0; % ft
t = 6.0; % ft
Iz = 40000; % lbs*ft^2
Ix = 15000; % lbs*ft^2
C = 0.5; % ft
dl_phi_f = 8000; % lbs*ft
dl_phi_r = 5000; % lbs*ft
dl_dphi_f = 1000; % lbs*ft
dl_dphi_r = 500; % lbs*ft

p = 12*in2ft; % ft
d = 12*in2ft; % ft
efficiency = 1.0; % Steering gearbox efficiency
Ks = 10*in2ft*rad2deg; % (in*lbs/deg)*(ft/in)*(deg/rad) -> ft*lbs / rad
tm = 3*in2ft; % in - Pneumatic trail

g = 32.174; % ft/sec^2

m = W / g;
ms = Ws / g;

u = 30; % ft/sec

C1 = 140*rad2deg;
C2 = 140*rad2deg;

dt = 0.01;
t_initial = 0;
t_final = 25;

%% 1. Develop a base 2-DoF linear state space yaw rate model and calculate the system eigen values
% when forward speed is 30 and 60 mph

u30 = 30*mph2ftps;
u60 = 60*mph2ftps;


A30 = [
    (-C1 - C2)/(m*u30), ((-x1*C1 - x2*C2)/(m*u30^2)) - 1;
    (-x1*C1 - x2*C2)/Iz, (-x1^2*C1 - x2^2*C2)/(Iz*u30);
];

A60 = [
    (-C1 - C2)/(m*u60), ((-x1*C1 - x2*C2)/(m*u60^2)) - 1;
    (-x1*C1 - x2*C2)/Iz, (-x1^2*C1 - x2^2*C2)/(Iz*u60);
];


eg_30mph = eig(A30);
eg_60mph = eig(A60);

%%  2. Using the steady-state yaw rate response, construct plots from 0 to 120 mph (i.e. 𝑟/𝛿 ) for various
% speeds, every 10 mph or so.

speeds = linspace(10, 120, 11);
speeds = speeds*mph2ftps;

% Time domain simulation:
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

% Generate a step input for the steering angle delta:
delta = zeros(1, length(t));
delta(1, length(t)/2:length(t)) = 0.1;

figure;
hold on;
ylabel('yaw rate (rad/sec)')
xlabel('time (sec)')
title(['yaw rate response for different speeds']);

for i = 1:length(speeds)
    u = speeds(i);
    states_arr = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta);

    % Plot yaw rate vs time, use speed as legend:
    plot(t, states_arr(2,:), 'linewidth', 2);
    % legend(num2str(speeds(i)*ftps2mph) + ' mph');
end


%% Run time domain simulation:

t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

% Generate a step input for the steering angle delta:
delta = zeros(1, length(t));
delta(1, length(t)/2:length(t)) = 0.1;

states_arr = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta);

% Time plots for each state:
figure
plot(t, states_arr(1,:));
figure
plot(t, states_arr(2,:));


function states_arr = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta)
    % Bicycle model using attack angle conventions:
    % Attack angle is the negative of the slip angle
    % The sign is embedded in the x1 and x2 distances
    
    A = [
        (-C1 - C2)/(m*u), ((-x1*C1 - x2*C2)/(m*u^2)) - 1;
        (-x1*C1 - x2*C2)/Iz, (-(x1^2)*C1 - (x2^2)*C2)/(Iz*u);
    ];
    
    B = [
        (C1)/(m*u);
        (x1*C1)/Iz;
    ];
    
    
    % Discretize using Euler:
    dt = 0.01; % seconds
    
    A_dis = eye(2) + dt * A;
    B_dis = dt * B;
    
    % Initial conditions:
    states = [0; 0];
    states_arr = zeros(2,length(t));
    
    % simulation loop:
    for i = 1:length(t)
        states = A_dis * states + B_dis * delta(i);
        states_arr(:, i) = states;
    end
end


