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
g = 32.174; % ft/sec^2
x1 = 3.5; % ft
x2 = -4.5; % ft = -1.0; % ft
h = -1.0;
track_width = 6.0; % ft
Iz = 40000 / g; % lbs*ft^2
Ix = 15000 / g; % lbs*ft^2
c = 0.5; % ft
dl_phi_f = 8000; % lbs*ft
dl_phi_r = 5000; % lbs*ft
dl_dphi_f = 1000; % lbs*ft
dl_dphi_r = 500; % lbs*ft

p = 12*in2ft; % ft
d = 12*in2ft; % ft
gear_ratio = 15; % []
efficiency = 1.0; % Steering gearbox efficiency
Ks = 10*in2ft*rad2deg; % (in*lbs/deg)*(ft/in)*(deg/rad) -> ft*lbs / rad
tm = 3*in2ft; % in*(ft/in)-> ft - Pneumatic trail

% Masses:
m = W / g;
ms = Ws / g;

% u = 30; % ft/sec

C1 = 140*180/pi; %*rad2deg; % lbs/deg * (deg/rad) -> lbs / rad
C2 = 140*180/pi; % *rad2deg; % lbs/deg * (deg/rad) -> lbs / rad

l2 = x1 - x2; % Wheelbase

dt = 0.01;
t_initial = 0;
t_final = 25;

%% 1. Develop a base 2-DoF linear state space yaw rate model and calculate the system eigen values
% when forward speed is 30 and 60 mph

u30 = 30*mph2ftps;
u60 = 60*mph2ftps;

A30 = [
    (-C1 - C2)/(m*u30), (((-x1*C1 - x2*C2)/(m*u30^2)) - 1);
    (-x1*C1 - x2*C2)/(Iz), (-x1*x1*C1 - x2*x2*C2)/(Iz*u30);
];

A60 = [
    (-C1 - C2)/(m*u60), ((-x1*C1 - x2*C2)/(m*u60^2)) - 1;
    (-x1*C1 - x2*C2)/(Iz), (-x1*x1*C1 - x2*x2*C2)/(Iz*u60);
];


eg_30mph = eig(A30);
eg_60mph = eig(A60);

%%  2. Using the steady-state yaw rate response, construct plots from 0 to 120 mph (i.e. 𝑟/𝛿 ) for various
% speeds, every 10 mph or so.

speeds = linspace(10, 120, 12);
speeds = speeds*mph2ftps;

% Time domain simulation:
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

% Generate a step input for the steering angle delta:
delta = zeros(1, length(t));
delta(1, length(t)/4:length(t)) = 0.02;

figure;
hold on;
ylabel('yaw rate (rad/sec)')
xlabel('time (sec)')
title(['yaw rate response for different speeds']);

legend_arr = cell(1,length(speeds));

for i = 1:length(speeds)
    u = speeds(i);
    states_arr = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta);

    % Plot yaw rate vs time, use speed as legend:
    plot(t, states_arr(2,:), 'linewidth', 2);
    legend_arr{i} = [num2str(u*ftps2mph)  ' mph']; % + ' mph';
end

legend(legend_arr{:});
grid;
hold off;

%% 3. Using the 2-DoF equations of motion, simulate the vehicle response to a steering input.
% Determine the steering input required to result in a 400 ft radius turn. (hint: knowing the forward
% speed and radius of a circle, the required yaw rate can be calculated). Repeat the result at
% increments of 10 mph from 10 to 120 mph. Compare the results with the steady-state results from
% 2) above. We are using this step to validate your model.

radius = 400; % ft
speeds = linspace(10, 120, 12);
speeds = speeds*mph2ftps;
figure;

% Time domain simulation:
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

for i = 1:length(speeds)
    u = speeds(i);

    delta1 = ones(1, length(t))*0.02;
    states_arr1 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta1);
    subplot(2,1,1);
    plot(t, states_arr1(2,:));
    xlabel('time (sec)');
    ylabel('yaw rate (rad/sec)');
    title(['yaw rate response for different speeds']);
    hold on;

    delta2 = (u / radius)*ones(1, length(t));
    states_arr2 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);
    subplot(2,1,2);
    plot(t, states_arr2(2,:));
    xlabel('time (sec)');
    ylabel('yaw rate (rad/sec)');
    title(['turn response for different speeds']);
    hold on;
end

subplot(2,1,1)
hold off;
subplot(2,1,2)
hold off;

%% 4. Determine a particular time series of handwheel inputs. We define this input as a 0° handwheel
% input for 1 second, then a +45° handwheel input for 3 seconds, then a −45° handwheel input for 3
% seconds, then back to 0° for 3 seconds. Practically we know that the maximum handwheel input a
% driver can perform is 2 𝑟𝑒𝑣/𝑠𝑒𝑐, so modify the steering input so that the handwheel velocity is
% consistent with this value with a 180° input amplitude. Plot the time history of the steering input 𝛿.

% Time array:
t = linspace(t_initial, t_final, (t_final - t_initial)/dt);

% Generate the specified handwheel input:
delta = zeros(1, length(t));
% Input of 0° for 1 second:
delta(1, 1:1/dt) = 0;
% Input of +45° for 3 seconds:
delta(1, 1/dt:4/dt) = 0.707;
% Input of -45° for 3 seconds:
delta(1, 4/dt:7/dt) = -0.707;
% Input of 0° for 3 seconds:
delta(1, 7/dt:length(t)) = 0;

% Plot input:
figure;
plot(t, delta);
grid on;
title('steering input with no rate saturation');
xlabel('time (sec)');
ylabel('delta (rad)');

% Modify the steering input:
delta_mod = delta; % Copy the original input
slope = 2*(2*pi); % rad/sec

% Smooth step applied at 1 second:
left_bound = 0;
right_bound = 0.707;
at = 1; % /dt;

delta_mod = slope_it_down(t, dt, at, delta, left_bound, right_bound, sign(right_bound - left_bound)*slope);

% Smooth step applied at 4 seconds:
left_bound = 0.707;
right_bound = -0.707;
at = 4; % /dt;

delta_mod = slope_it_down(t, dt, at, delta_mod, left_bound, right_bound, sign(right_bound - left_bound)*slope);

% Smooth step applied at 7 seconds:
left_bound = -0.707;
right_bound = 0;
at = 7; % /dt;

delta_mod = slope_it_down(t, dt, at, delta_mod, left_bound, right_bound, sign(right_bound - left_bound)*slope);

% Plot input:
figure;
plot(t, delta);
hold on;
plot(t, delta_mod, 'r');
grid on;
title('steering input with smoothed angle transitions');
xlabel('time (sec)');
ylabel('delta (rad)');
legend('Original input signal', 'Modified input signal');

%% 5. Use the steering input calculated in 4) as an input to your simulation. Plot the time history of the
% state variables drift angle 𝛽 and yaw rate 𝑟 at 30 and 60 mph.

% Speeds of interest:
u30 = 30*mph2ftps;
u60 = 60*mph2ftps;

% Time domain simulation:
states_arr30 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u30, delta_mod);
states_arr60 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u60, delta_mod);

% Plot the states:
figure;
subplot(2,1,1);
plot(t, states_arr30(1,:));
xlabel('time (sec)');
ylabel('drift angle (rad)');
title('yaw rate for 30 mph');
hold on;
plot(t, states_arr60(1,:));
xlabel('time (sec)');
ylabel('drift angle (rad)');
title('yaw rate for 60 mph');
legend('30 mph', '60 mph');


subplot(2,1,2);
plot(t, states_arr30(2,:));
xlabel('time (sec)');
ylabel('yaw rate (rad/sec)');  
hold on;
plot(t, states_arr60(2,:));
title('yaw rate for 30 and 60 mph');
legend('30 mph', '60 mph');


%% 6. Add nonlinear tires and assume equal roll moment front and rear, 60/40 biased to the front, and
% 40/60 biased to the rear. Compare your biased results to the results of 3)-5) with linear tires.

% % 50 / 50 weight bias:
% W1 = W / 2; W2 = W / 2;
% C1 = (0.2*W1 - 0.0000942*(W1^2))*deg2rad; % lbs/rad
% C2 = (0.2*W2 - 0.0000942*(W2^2))*deg2rad; % lbs/rad

% states_arr_50_50 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta);

% % 60 / 40 weight bias:
% W1 = 0.6*W; W2 = 0.4*W;
% C1 = (0.2*W1 - 0.0000942*(W1^2))*deg2rad; % lbs/rad
% C2 = (0.2*W2 - 0.0000942*(W2^2))*deg2rad; % lbs/rad

% states_arr_60_40 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta);

% % 40 / 60 weight bias:
% W1 = 0.4*W; W2 = 0.6*W;
% C1 = (0.2*W1 - 0.0000942*(W1^2))*deg2rad; % lbs/rad
% C2 = (0.2*W2 - 0.0000942*(W2^2))*deg2rad; % lbs/rad

% states_arr_40_60 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta);

%% Compare with 3):
delta1 = ones(1, length(t))*0.02;
delta2 = (u / radius)*ones(1, length(t));

speeds = linspace(10, 120, 12);
speeds = speeds*mph2ftps;

% Time domain simulation:
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);


figure(1); % For delta 1
hold on;
legends = {};

for i = 1:length(speeds)
    u = speeds(i);

    C1 = 140*deg2rad;
    C2 = 140*deg2rad;
    states_arr_lin = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta1);

    % 50 / 50 weight bias:
    W1 = W / 2; W2 = W / 2;
    C1 = (0.2*W1 - 0.0000942*(W1^2))*deg2rad; % lbs/rad
    C2 = (0.2*W2 - 0.0000942*(W2^2))*deg2rad; % lbs/rad
    states_arr_50_50 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta1);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', 2);
    legends{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_50_50(2,:), 'linewidth', 2);
    legends{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 50-50 wb'];    

    % print u:
    disp(['u = ' num2str(u*ftps2mph) ' mph']);

    % states_arr2 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);
    % subplot(4,1,2);
    % plot(t, states_arr2(2,:));
    % xlabel('time (sec)');
    % ylabel('yaw rate (rad/sec)');
    % title(['turn response for different speeds']);
    % hold on;
end

legend(legends{:});
hold off;

% Plot the states:
% figure;
% subplot(2,1,1);
% plot(t, states_arr_50_50(1,:));
% xlabel('time (sec)');
% ylabel('drift angle (rad)');
% title('yaw rate for 50/50 weight bias');
% hold on;
% plot(t, states_arr_60_40(1,:));
% xlabel('time (sec)');
% ylabel('drift angle (rad)');
% title('yaw rate for 60/40 weight bias');
% hold on;
% plot(t, states_arr_40_60(1,:));
% xlabel('time (sec)');
% ylabel('drift angle (rad)');
% title('yaw rate for 40/60 weight bias');
% legend('50/50', '60/40', '40/60');
% 
% subplot(2,1,2);
% plot(t, states_arr_50_50(2,:));
% xlabel('time (sec)');
% ylabel('yaw rate (rad/sec)');
% title('yaw rate for 50/50 weight bias');
% hold on;
% plot(t, states_arr_60_40(2,:));
% xlabel('time (sec)');
% ylabel('yaw rate (rad/sec)');
% title('yaw rate for 60/40 weight bias');
% hold on;
% plot(t, states_arr_40_60(2,:));
% xlabel('time (sec)');
% ylabel('yaw rate (rad/sec)');
% title('yaw rate for 40/60 weight bias');
% legend('50/50', '60/40', '40/60');


%% Try out transfer functions:

u = speeds(5);
K_understeer = -m*(x1*C1 + x2*C2)/(C1*C2*l2);
G_yaw_rate = tf([((x1*m*u*u)/(l2*C2)), u], [(m*u*u*Iz)/(l2*C1*C2), (u/l2)*((m*x1*x1+Iz)/(C2)), l2 + u*u*K_understeer]);

step(G_yaw_rate)


%% Run time domain simulation:
t_initial = 0;
t_final = 25;
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

% Generate a step input for the steering angle delta:
delta = zeros(1, length(t));
delta(1, length(t)/4:length(t)) = 0.1;

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
        (-x1*C1 - x2*C2)/(Iz), (-(x1^2)*C1 - (x2^2)*C2)/(Iz*u);
    ];
    
    B = [
        (C1)/(m*u);
        (x1*C1)/Iz;
    ];
    
    
    % Discretize using Euler:    
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


function delta = slope_it_down(t, dt, at, delta, left_bound, right_bound, slope)

    time_to_reach = (right_bound - left_bound) / slope; % time units
    delta(1, at/dt:(at + time_to_reach)/dt) = left_bound + slope * (t(at/dt:(at + time_to_reach)/dt) - t(at/dt)); % * sign(right_bound - left_bound);

end


