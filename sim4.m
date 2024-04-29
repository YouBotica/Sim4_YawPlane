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
h = -1.0; % ft
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

C1 = 2*140*180/pi; %*rad2deg; % lbs/deg * (deg/rad) -> lbs / rad
C2 = 2*140*180/pi; % *rad2deg; % lbs/deg * (deg/rad) -> lbs / rad

l2 = x1 - x2; % Wheelbase

dt = 0.01;
t_initial = 0;
t_final = 5;

%% 1. Develop a base 2-DoF linear state space yaw rate model and calculate the system eigen values
% when forward speed is 30 and 60 mph

u30 = 30*mph2ftps;
u60 = 60*mph2ftps;

A30 = [
    (-C1 - C2)/(m*u30), (((-x1*C1 - x2*C2)/(m*u30^2)) - 1);
    (-x1*C1 - x2*C2)/(Iz), (-x1*x1*C1 - x2*x2*C2)/(Iz*u30);
]

A60 = [
    (-C1 - C2)/(m*u60), ((-x1*C1 - x2*C2)/(m*u60^2)) - 1;
    (-x1*C1 - x2*C2)/(Iz), (-x1*x1*C1 - x2*x2*C2)/(Iz*u60);
]



eg_30mph = eig(A30)
eg_60mph = eig(A60)

%%  2. Using the steady-state yaw rate response, construct plots from 0 to 120 mph (i.e. 𝑟/𝛿 ) for various
% speeds, every 10 mph or so.


speeds = linspace(10, 120, 12)*mph2ftps;

delta2r_gain_arr = zeros(1, length(speeds));

t = linspace(t_initial, t_final, (t_final - t_initial) / dt);


figure; 
hold on;
grid on;
title('Yaw rate steady state gains for different speeds')
xlabel('Time (seconds)');
ylabel('Gain (𝑟/𝛿)');
xlim([0, 5]);
ylim([0, 10.0]);

K_understeer = -m*(x1*C1 + x2*C2)/(C1*C2*l2);
u_char = sqrt((x1-x2)/K_understeer);

legend_arr = cell(1, length(speeds)); 
for i = 1:length(speeds)
    u = speeds(i);
    delta2r_gain_arr(i) = u / (l2 + u*u*K_understeer);

    % plot each gain in an xy plot:
    yline(delta2r_gain_arr(i),'--', ['s.s gain = ' num2str(delta2r_gain_arr(i)) ' @ ' num2str(u*ftps2mph) 'mph'], 'Color', rand(1,3), 'LineWidth', 2);
    % plot(t, delta2r_gain_arr(i)*ones(length(t)), '--', 'LineWidth', 2);
    legend_arr{i} = [num2str(u*ftps2mph) ' mph'];
    hold on;
end

legend(legend_arr{:}, 'Location', 'eastoutside');
hold off;


%% 3. Using the 2-DoF equations of motion, simulate the vehicle response to a steering input.
% Determine the steering input required to result in a 400 ft radius turn. (hint: knowing the forward
% speed and radius of a circle, the required yaw rate can be calculated). Repeat the result at
% increments of 10 mph from 10 to 120 mph. Compare the results with the steady-state results from
% 2) above. We are using this step to validate your model.

radius = 400; % ft
speeds = linspace(10, 120, 12);
speeds = speeds*mph2ftps;

% Time domain simulation:
t_initial = 0; % sec
t_final = 5; % sec
dt = 0.01; % sec
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

figure;
subplot(3,1,1);
title('Steady state gains at different speeds');
xlabel('Time (seconds)');
ylabel('Gain []');
hold on;
grid on;

subplot(3,1,2);
title('Yaw rate response at different speeds');
xlabel('Time (seconds)');
ylabel('Yaw rate (rad/sec)');
hold on;

subplot(3,1,3);
title('Steering angle input at different speeds');
xlabel('Time (seconds)');
ylabel('Steering angle (rad)');
hold on;

legend_gains_arr = cell(1, length(speeds));
legend_yaw_rate_arr = cell(1, length(speeds));
legend_steering_angle_arr = cell(1, length(speeds));

for i = 1:length(speeds)
    u = speeds(i);

    subplot(3,1,1);
    yline(delta2r_gain_arr(i),'--', ['s.s gain = ' num2str(delta2r_gain_arr(i)) ' @ ' num2str(u*ftps2mph) 'mph'], 'Color', rand(1,3), 'LineWidth', 2);
    % plot(t, delta2r_gain_arr(i)*ones(length(t)), '--', 'LineWidth', 2);
    legend_gains_arr{i} = ['s.s gain = ' num2str(delta2r_gain_arr(i)) ' @ ' num2str(u*ftps2mph) 'mph'];
    
    subplot(3,1,2);
    delta2 = (1 / radius)*(l2 + (u*u*K_understeer))*ones(1, length(t));
    states_arr2 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);
    plot(t, states_arr2(2,:));
    legend_yaw_rate_arr{i} = ['Calculated s.s gain @ ' num2str(u*ftps2mph) ' is ' num2str(states_arr2(2, length(t))/delta2(length(t)))];

    subplot(3,1,3);
    plot(t, delta2);
    legend_steering_angle_arr{i} = ['𝛿 = ' num2str(delta2(1)) ' @ ' num2str(u*ftps2mph) 'mph'];
end


subplot(3,1,1);
legend(legend_gains_arr);
grid on;

subplot(3,1,2);
legend(legend_yaw_rate_arr);
grid on;

subplot(3,1,3);
legend(legend_steering_angle_arr);
grid on;

hold off;

%% 4. Determine a particular time series of handwheel inputs. We define this input as a 0° handwheel
% input for 1 second, then a +45° handwheel input for 3 seconds, then a −45° handwheel input for 3
% seconds, then back to 0° for 3 seconds. Practically we know that the maximum handwheel input a
% driver can perform is 2 𝑟𝑒𝑣/𝑠𝑒𝑐, so modify the steering input so that the handwheel velocity is
% consistent with this value with a 180° input amplitude. Plot the time history of the steering input 𝛿.

% Time array:
t_initial = 0;
t_final = 10;
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
title('Steering input with no rate saturation');
xlabel('time (sec)');
ylabel('delta (rad)');

% Modify the steering input:
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
title('Steering input with smoothed angle transitions');
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
grid on;
% title('yaw rate for 30 mph');
hold on;
plot(t, states_arr60(1,:));
xlabel('time (sec)');
ylabel('drift angle (rad)');
title('Drift angle for 30 and 60 mph');;
legend('30 mph', '60 mph');


subplot(2,1,2);
plot(t, states_arr30(2,:));
xlabel('time (sec)');
ylabel('yaw rate (rad/sec)');  
grid on;
hold on;
plot(t, states_arr60(2,:));
title('Yaw rate for 30 and 60 mph');
legend('30 mph', '60 mph');


%% 6. Add nonlinear tires and assume equal roll moment front and rear, 60/40 biased to the front, and
% 40/60 biased to the rear. Compare your biased results to the results of 3)-5) with linear tires.

%% Compare with 3):
close all;

speeds = linspace(10, 120, 12);
speeds = speeds*mph2ftps;

% Time domain simulation:
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);


figure(1); % 50-50 weight bias
hold on; 
figure(2); % 60-40 weight bias
hold on;
figure(3);
hold on;

legends_50_50 = {};
legends_60_40 = {};
legends_40_60 = {};


for i = 1:length(speeds)
    u = speeds(i);
    delta2 = (u / radius)*ones(1, length(t)); % Update delta to accomplish 400 ft radius
    
    % Linear tire:
    C1 = 2*140*180/pi;
    C2 = 2*140*180/pi;
    states_arr_lin = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);

    
    % 50 / 50 weight bias:
    figure(1);
    W1 = W / 2; W2 = W / 2;
    C1 = 2*(0.2*W1 - 0.0000942*(W1^2))*180/pi; % lbs/rad
    C2 = 2*(0.2*W2 - 0.0000942*(W2^2))*180/pi; % lbs/rad
    states_arr_50_50 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', 2);
    legends_50_50{end+1} = ['Yaw rate @' num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_50_50(2,:), 'linewidth', 2);
    legends_50_50{end+1} = [num2str(u*ftps2mph) ' mph with non-linear tires and 50-50 wb'];
    K_understeer_50_50 = -m*(x1*C1 + x2*C2)/(C1*C2*l2);
    u_char_50_50 = sqrt(l2/K_understeer_50_50);


    % 60 / 40 weight bias:
    figure(2);

    W1 = 0.6*W; W2 = 0.4*W;
    C1 = 2*(0.2*W1 - 0.0000942*(W1^2))*180/pi; % lbs/rad
    C2 = 2*(0.2*W2 - 0.0000942*(W2^2))*180/pi; % lbs/rad
    states_arr_60_40 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', 2);
    legends_60_40{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_60_40(2,:), 'linewidth', 2);
    legends_60_40{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 60-40 wb']; 
    K_understeer_60_40 = -m*(x1*C1 + x2*C2)/(C1*C2*l2);
    u_char_60_40 = sqrt(l2/K_understeer_60_40);


    % 40 / 60 weight bias:
    figure(3);

    W1 = 0.4*W; W2 = 0.6*W;
    C1 = 2*(0.2*W1 - 0.0000942*(W1^2))*180/pi; % lbs/rad
    C2 = 2*(0.2*W2 - 0.0000942*(W2^2))*180/pi; % lbs/rad
    states_arr_40_60 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', 2);
    legends_40_60{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_40_60(2,:), 'linewidth', 2);
    legends_40_60{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 40-60 wb']; 
    K_understeer_40_60 = -m*(x1*C1 + x2*C2)/(C1*C2*l2);
    u_critical_40_60 = sqrt(l2/-K_understeer_40_60);

end


figure(1);
legend(legends_50_50{:});
% Add text to plot with understeering coefficient and characteristic speed:
text(4.0, 4.0, ['K_{understeer} = ' num2str(K_understeer_50_50) ' u_{char} = ' num2str(u_char_50_50)], 'FontSize', 16);
hold off;
figure(2);
legend(legends_60_40{:});
% Add text to plot with understeering coefficient and characteristic speed:
text(4.0, 4.0, ['K_{understeer} = ' num2str(K_understeer_60_40) ' u_{char} = ' num2str(u_char_60_40)], 'FontSize', 16);
hold off;
figure(3);
legend(legends_40_60{:});
% Add text to plot with understeering coefficient and critical speed:
text(4.0, 1.5e9, ['K_{understeer} = ' num2str(K_understeer_40_60) ' u_{critical} = ' num2str(u_critical_40_60)], 'FontSize', 16);
hold off;

%% Compare with 4-5:
close all;

line_width = 1;

speeds = linspace(10, 120, 12);
speeds = speeds*mph2ftps;

% Time domain simulation:
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);


figure(1); % 50-50 weight bias
subplot(2,1,1);
hold on; 
xlabel('Time (seconds)');
ylabel('Yaw rate (rad/sec)')
title('Yaw rate time response 50/50 bias');
subplot(2,1,2);
hold on; 
xlabel('Time (seconds)');
ylabel('Drift angle (rad)');
title('Drift angle time response 50/50 bias');

figure(2); % 60-40 weight bias
subplot(2,1,1);
hold on; 
xlabel('Time (seconds)');
ylabel('Yaw rate (rad/sec)');
title('Yaw rate time response 60/40 bias');
subplot(2,1,2);
hold on; 
xlabel('Time (seconds)');
ylabel('Drift angle (rad)');
title('Drift angle time response 60/40 bias');

figure(3);
subplot(2,1,1);
hold on; 
xlabel('Time (seconds)');
ylabel('Yaw rate (rad/sec)');
title('Yaw rate time response 40/60 bias');

subplot(2,1,2);
hold on; 
xlabel('Time (seconds)');
ylabel('Drift angle (rad)');
title('Drift angle time response 40/60 bias');

legends_50_50 = {};
legends_60_40 = {};
legends_40_60 = {};


for i = 1:length(speeds)
    u = speeds(i);

    % Linear tire:
    C1 = 2*140*180/pi;
    C2 = 2*140*180/pi;
    states_arr_lin = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta_mod);

    % Non-linear tires:

    % 50 / 50 weight bias:
    figure(1);

    W1 = W / 2; W2 = W / 2;
    C1 = 2*(0.2*W1 - 0.0000942*(W1^2))*180/pi; % lbs/rad
    C2 = 2*(0.2*W2 - 0.0000942*(W2^2))*180/pi; % lbs/rad
    states_arr_50_50 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta_mod);

    subplot(2,1,1);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', line_width);
    legends_50_50{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_50_50(2,:), 'linewidth', line_width);
    legends_50_50{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 50-50 wb'];  

    subplot(2,1,2);
    plot(t, states_arr_lin(1,:), '--', 'linewidth', line_width);
    legends_50_50{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_50_50(1,:), 'linewidth', line_width);
    legends_50_50{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 50-50 wb'];  


    % 60 / 40 weight bias:
    figure(2);

    W1 = 0.6*W; W2 = 0.4*W;
    C1 = 2*(0.2*W1 - 0.0000942*(W1^2))*180/pi; % lbs/rad
    C2 = 2*(0.2*W2 - 0.0000942*(W2^2))*180/pi; % lbs/rad
    states_arr_60_40 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta_mod);

    subplot(2,1,1);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', line_width);
    legends_60_40{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_60_40(2,:), 'linewidth', line_width*2);
    legends_60_40{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 60-40 wb']; 


    % 40 / 60 weight bias:
    figure(3);

    W1 = 0.4*W; W2 = 0.6*W;
    C1 = 2*(0.2*W1 - 0.0000942*(W1^2))*180/pi; % lbs/rad
    C2 = 2*(0.2*W2 - 0.0000942*(W2^2))*180/pi; % lbs/rad
    states_arr_40_60 = simulate_bike_2dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta_mod);
    plot(t, states_arr_lin(2,:), '--', 'linewidth', line_width);
    legends_40_60{end+1} = [num2str(u*ftps2mph) ' mph with linear tires'];
    plot(t, states_arr_40_60(2,:), 'linewidth', line_width);
    legends_40_60{end+1} = [num2str(u*ftps2mph) ' mph with non-linear and 40-60 wb']; 

end

figure(1);
legend(legends_50_50{:});
hold off;
figure(2);
legend(legends_60_40{:});
hold off;
figure(3);
legend(legends_40_60{:});
hold off;


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


