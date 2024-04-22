%% Load some initial values:

% Conversion factors:
deg2rad = pi / 180;
rad2deg = 180 / pi;
in2ft = 1 / 12;
ft2in = 12;
mph2ftps = 5280 / 3600;
ftps2mph = 3600 / 5280;

% Bicycle model parameters:
W = 1000; % lbs
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

p = 12*in2ft; % in 
d = 12*in2ft; % in
efficiency = 1.0; % Steering gearbox efficiency
Ks = 10*in2ft*rad2deg; % (in*lbs/deg)*(ft/in)*(deg/rad) -> ft*lbs / rad
tm = 3; % in - Pneumatic trail

g = 32.174; % ft/sec^2

m = W / g;
ms = Ws / g;

vx = 30; % ft/sec

C1 = 140*rad2deg;
C2 = 140*rad2deg;

dt = 0.01;
t_initial = 0;
t_final = 25;

% Generate a step input for the steering angle delta:
delta = zeros(1, t_final / dt);

states_arr = simulate_bike_2dof(dt, t_initial, t_final, m, x1, x2, C1, C2, Iz, vx, delta);



function states_arr = simulate_bike_2dof(dt, t_initial, t_final, m, x1, x2, C1, C2, Iz, u, delta)
    %% Bicycle model using attack angle conventions:
    % Attack angle is the negative of the slip angle
    % The sign is embedded in the x1 and x2 distances
    
    A = [
        (-C1 - C2)/(m*u), ((-x1*C1 - x2*C2)/(m*u)) - u;
        (-x1*C1 - x2*C2)/Iz, (-x1^2*C1 - x2^2*C2)/Iz;
    ];
    
    B = [
        (C1)/(m*u);
        (x1*C1)/Iz;
    ];
    
    
    % Discretize using Euler:
    dt = 0.01; % seconds
    
    A_dis = eye(2) + dt * A;
    B_dis = dt * B;
    
    t = linspace(t_initial, t_final, (t_final - t_initial) / dt);
    
    % Initial conditions:
    states = [0; 0];
    states_arr = zeros(2,length(t));
    
    % simulation loop:
    for i = 1:length(t)
        states = A_dis * states + B_dis * delta(i);
        states_arr(:, i) = states;
    end
end


