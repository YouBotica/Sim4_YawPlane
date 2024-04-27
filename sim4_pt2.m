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

%% 1. Repeat the steps in (Part 1: 2-DoF Vehicle) with a three degree of freedom model allowing sprung
% mass roll. Using the steady-state yaw rate response, construct plots from 0 to 120 mph (i.e. 𝛿 ) for
% various speeds without roll steer and with a rear roll steer coefficient of 𝜖𝑟 = −0.03.

% Calculate the steady state gains for different speeds:

speeds = linspace(10, 120, 12)*mph2ftps;

delta2r_gain_arr = zeros(1, length(speeds));

dt = 0.01;
t_initial = 0;
t_final = 5;

t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

figure; 
hold on;
grid on;
xlabel('Time (seconds)');
ylabel('Gain (𝑟/𝛿)');
xlim([t_initial, t_final]);
ylim([0, 6.0]);

% TODO: Calculate K_understeer

% Roll steer:
eps1 = 0;
eps2 = -0.03;

C_phi1 = C1*eps1; C_phi2 = C2*eps2;
Ca = C1 + C2;
Cb = x1*C1 + x2*C2;
Cc = x1*x1*C1 + x2*x2*C2;

% Roll stiffness:
K_phi = (dl_phi_f + dl_phi_f + ms*g*h); % lb*ft / rad
% Roll damping:
D_phi = (dl_dphi_f + dl_dphi_r); % lb*ft/sec / (rad/sec)


legend_arr = cell(1, length(speeds)); 
for i = 1:length(speeds)
    u = speeds(i);

    % K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
    % K_roll_effect = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

    % K_understeer = K_understeer_wout_roll + K_roll_effect;
    % 
    % delta2r_gain_arr(i) = (u) / (l2 + u*u*K_understeer);
    
    delta2r_gain_arr(i) = (u*C1*(Cb-x1*Ca)) /(Cb*Cb - Ca*Cc + Cb*m*u*u + (x1*Ca*C_phi1 + x2*Ca*C_phi2 - (C_phi1 + C_phi2)*Cb)*(ms*h*u*u / K_phi));

    % plot each gain in an xy plot:
    yline(delta2r_gain_arr(i),'--', ['s.s gain = ' num2str(delta2r_gain_arr(i)) ' @ ' num2str(u*ftps2mph) 'mph'], 'Color', rand(1,3), 'LineWidth', 2);
    % plot(t, delta2r_gain_arr(i)*ones(length(t)), '--', 'LineWidth', 2);
    legend_arr{i} = [num2str(u*ftps2mph) ' mph'];
    hold on;
end

legend(legend_arr{:});

hold off;

%% 2. Using the 3-DoF equations of motion, simulate the vehicle response to a steering input.
% Determine the steering input required to result in a 400 𝑓𝑡 radius turn (hint: knowing the forward
% speed and radius of a circle, the required yaw rate can be calculated). Repeat the result at
% increments of 10 mph from 10 to 120 mph. Compare the results with the steady-state results from
% 1) above. As before, we are using this step to validate your simulation model.

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
xlabel('Time (seconds)');
ylabel('Gain []');
hold on;
grid on;

subplot(3,1,2);
xlabel('Time (seconds)');
ylabel('Yaw rate (rad/sec)');
hold on;

subplot(3,1,3);
xlabel('Time (seconds)');
ylabel('Steering angle (rad)');
hold on;

legend_gains_arr = cell(1, length(speeds));
legend_yaw_rate_arr = cell(1, length(speeds));
legend_steering_angle_arr = cell(1, length(speeds));

eps1 = 0; eps2 = -0.03;

for i = 1:length(speeds)
    u = speeds(i);

    % K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
    % K_roll_effect = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);
    
    delta2r_gain_arr(i) = (u*C1*(Cb-x1*Ca)) /(Cb*Cb - Ca*Cc + Cb*m*u*u + (x1*Ca*C_phi1 + x2*Ca*C_phi2 - (C_phi1 + C_phi2)*Cb)*(ms*h*u*u / K_phi));

    subplot(3,1,1);
    % plot(t, delta2r_gain_arr(i)*ones(length(t)), '--', 'LineWidth', 2);     % DEPRECATED

    yline(delta2r_gain_arr(i),'--', ['s.s gain = ' num2str(delta2r_gain_arr(i)) ' @ ' num2str(u*ftps2mph) 'mph'], 'Color', rand(1,3), 'LineWidth', 2);
    legend_gains_arr{i} = ['s.s gain = ' num2str(delta2r_gain_arr(i)) ' @ ' num2str(u*ftps2mph) 'mph'];
    
    subplot(3,1,2);
    delta2 = (u / radius)*ones(1, length(t));

    % Simulate:
    states_arr2 = simulate_bike_3dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta2, ms, h, K_phi, D_phi, eps1, eps2, Ix, c);
    plot(t, states_arr2(2,:));
    legend_yaw_rate_arr{i} = ['Calculated s.s gain @ ' num2str(u*ftps2mph) ' is ' num2str(states_arr2(2, length(t))/delta2(length(t)))];

    subplot(3,1,3);
    plot(t, delta2);
    legend_steering_angle_arr{i} = ['𝛿 = ' num2str(delta2(1)) ' @ ' num2str(u*ftps2mph) 'mph'];
end


subplot(3,1,1);
hold off;
legend(legend_gains_arr);
grid on;

subplot(3,1,2);
hold off;
legend(legend_yaw_rate_arr);
grid on;

subplot(3,1,3);
hold off;
legend(legend_steering_angle_arr);
grid on;

hold off;

%% 3. Use the steering input calculated in 4) from Part 1 as an input to your simulation. Repeat results
% for the following combinations of roll steer coefficients:

% Generate the input: 

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
title('steering input with no rate saturation');
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
title('steering input with smoothed angle transitions');
xlabel('time (sec)');
ylabel('delta (rad)');
legend('Original input signal', 'Modified input signal');

%% Configuration A: 
eps1 = 0.04; eps2 = 0.04;

K_understeer_wout_roll = (-m*Cb/(C1*C2*l2));
K_roll_effectA = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerA = K_understeer_wout_roll + K_roll_effectA;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration B:
eps1 = 0.04; eps2 = 0.0;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*Cb/(C1*C2*l2));
K_roll_effectB = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerB = K_understeer_wout_roll + K_roll_effectB;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration C:
eps1 = 0.04; eps2 = -0.04;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*Cb/(C1*C2*l2));
K_roll_effectC = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerC = K_understeer_wout_roll + K_roll_effectC;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration D:
eps1 = 0; eps2 = 0.04;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
K_roll_effectD = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerD = K_understeer_wout_roll + K_roll_effectD;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration E:
eps1 = 0; eps2 = 0;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
K_roll_effectE= (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerE = K_understeer_wout_roll + K_roll_effectE;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration F:
eps1 = 0; eps2 = -0.04;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
K_roll_effectF = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerF = K_understeer_wout_roll + K_roll_effectF;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration G:
eps1 = -0.04; eps2 = 0.04;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
K_roll_effectG = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerG = K_understeer_wout_roll + K_roll_effectG;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);


%% Configuration H:
eps1 = -0.04; eps2 = 0;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
K_roll_effectH = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerH = K_understeer_wout_roll + K_roll_effectH;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Configuration I:
eps1 = -0.04; eps2 = -0.04;
desired_radius = 400;

C_phi1 = eps1*C1; C_phi2 = eps2*C2;

K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
K_roll_effectI = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);

K_understeerI = K_understeer_wout_roll + K_roll_effectI;

test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta_mod);

%% Part 3:
% Return to the baseline case of 𝜖𝑓 = 0; 𝜖𝑟 = −0.03. Add the nonlinear tire model. You will have to
% calculate the tire loads at each time step, knowing that the moments from roll stiffness and roll
% damping on the sprung mass are reacted by vertical forces through the contact patch of the tires,
% and add to or subtract from the static tire loads. Compare and interpret your results above with
% the corresponding results from (Part 2: 3-DoF Vehicle (Linear)).

eps1 = 0; eps2 = -0.03;

% C_phi1 = eps1*C1; C_phi2 = eps2*C2;

test_nonlinear_model(dt, t, m, x1, x2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta, W)

%% Test non-linear model:

% Generate a step test input for the steering angle delta:
delta_test = zeros(1, length(t));
delta_test(1, length(t)/4:length(t)) = 0.01;

% Simulate:
[W1l_arr, W1r_arr, W2l_arr, W2r_arr, states_arr] = simulate_non_linear_bike_3dof(dt, t, m, x1, x2, Iz, u, delta_test, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, W);

% Plot the results:
figure;
subplot(4,1,1);
plot(t, states_arr(2,:));
xlabel('Time (seconds)');
ylabel('Yaw rate (rad/sec)');
title('Yaw rate response for ε1= 0 and ε2= -0.03');
grid on;

subplot(4,1,2);
delta_test = zeros(1, length(t));
plot(t, states_arr(1,:) / u);
xlabel('Time (seconds)');
ylabel('Drift angle (rad)');
title('Drift angle response for ε1= 0, ε2= -0.03');
grid on;

subplot(4,1,3);
plot(t, states_arr(4,:));
xlabel('Time (seconds)');
ylabel('Roll (rad)');
title('Roll angle response for ε1= 0, ε2= -0.03');
grid on;

subplot(4,1,4);
plot(t, delta_mod);
xlabel('Time (seconds)');
ylabel('Steering angle (rad)');
title('Steering angle input');
grid on;


% test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi,
% eps1, eps2, Ix, c, delta_mod); TODO



%% Test linear model here:

t_initial = 0;
t_final = 5;
t = linspace(t_initial, t_final, (t_final - t_initial) / dt);

% Generate a step input for the steering angle delta:
delta_test = zeros(1, length(t));
delta_test(1, length(t)/4:length(t)) = 0.01;

states_arr = simulate_bike_3dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta_test, ms, h, K_phi, D_phi, eps1, eps2, Ix, c);

% Time plots for each state:
figure
plot(t, states_arr(1,:));
figure
plot(t, states_arr(2,:));



%% Build Yaw Plane Model with roll:


function states_arr = simulate_bike_3dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta, ms, h, K_phi, D_phi, eps1, eps2, Ix, c)
    % Bicycle model using attack angle conventions + roll as a 3rd degree of freedom:
    % Attack angle is the negative of the slip angle
    % The sign is embedded in the x1 and x2 distances

    % Ordinary bicycle model:
    Ca = C1 + C2;
    Cb = x1*C1 + x2*C2;
    Cc = x1*x1*C1 + x2*x2*C2;
    
    % Roll steer effects:
    C_phi1 = C1*eps1; C_phi2 = C2*eps2;

    % Parallel axis theorem:
    Ix_steiner = Ix + ms*h*h;

    A = [
        -Ca/u, -Cb/u - m*u, 0, C_phi1 + C_phi2;
        -Cb/u, -Cc/u, 0, x1*C_phi1 + x2*C_phi2;
        0, ms*h*u, -D_phi, -K_phi;  
    ];
    
    B = [
        C1;
        x1*C1;
        0; 
    ];

    inertia_matrix = [
        m, 0, -ms*h;
        0, Iz, -ms*h*c;
        -ms*h, -ms*h*c, Ix_steiner;
    ];
    
    
    % Discretize using Euler:    
    
    % Initial conditions:
    states = [0; 0; 0; 0];
    states_arr = zeros(length(states),length(t));
    
    % simulation loop:
    for i = 1:length(t)
        states(1:3) = (states(1:3) + inv(inertia_matrix)*dt*(A*states + B*delta(i))); % inv(inertia_matrix)*(A_dis * states + B_dis * delta(i));
        states(4) = states(4) + dt*states(3); % Roll = Roll_prev + dt*(dRoll/dt)
        states_arr(:, i) = states;
    end
end

function [W1l_arr, W1r_arr, W2l_arr, W2r_arr, states_arr] = simulate_non_linear_bike_3dof(dt, t, m, x1, x2, Iz, u, delta, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, W)
    % Bicycle model using attack angle conventions + roll as a 3rd degree of freedom:
    % Attack angle is the negative of the slip angle
    % The sign is embedded in the x1 and x2 distances

    % Parallel axis theorem:
    Ix_steiner = Ix + ms*h*h;

    inertia_matrix = [
        m, 0, -ms*h;
        0, Iz, -ms*h*c;
        -ms*h, -ms*h*c, Ix_steiner;
    ];
    
    % Initial conditions:
    states = [0; 0; 0; 0];
    states_arr = zeros(length(states),length(t));

    % Weight arrays:
    W1r_arr = zeros(length(t));
    W1l_arr = zeros(length(t));
    W2r_arr = zeros(length(t));
    W2l_arr = zeros(length(t));
    
    % simulation loop:[W1l_arr, W1r_arr, W2l_arr, W2r_arr, states_arr]
    for i = 1:length(t)
        
        W1 = W/2; W2 = W/2; % Assuming 50/50 weight bias:
        % Weight transfer:
        W1r = (W1/2) + (1/(4*h))*(-K_phi*states(4) - D_phi*states(3));
        W1l = (W1/2) - (1/(4*h))*(-K_phi*states(4) - D_phi*states(3));

        W2r = (W2/2) + (1/(4*h))*(-K_phi*states(4) - D_phi*states(3));
        W2l = (W2/2) - (1/(4*h))*(-K_phi*states(4) - D_phi*states(3));
        
        % Store the weights to analyze weight transfer over time:
        W1r_arr(i) = W1r;
        W1l_arr(i) = W1l;
        W2r_arr(i) = W2r;
        W2l_arr(i) = W2l;

        % Non-linear tires:
        C1r = 0.2*W1r - 0.0000942*W1r*W1r;
        C1l = 0.2*W1l - 0.0000942*W1l*W1l;

        C1 = C1r + C1l;

        C2r = 0.2*W2r - 0.0000942*W2r*W2r;
        C2l = 0.2*W2l - 0.0000942*W2l*W2l;

        C2 = C2r + C2l;

        % Roll steer effects:
        C_phi1 = C1*eps1; C_phi2 = C2*eps2;
    
        % Bicycle model with roll:
        Ca = C1 + C2;
        Cb = x1*C1 + x2*C2;
        Cc = x1*x1*C1 + x2*x2*C2;

        A = [
            -Ca/u, -Cb/u - m*u, 0, C_phi1 + C_phi2;
            -Cb/u, -Cc/u, 0, x1*C_phi1 + x2*C_phi2;
            0, ms*h*u, -D_phi, -K_phi;  
        ];
    
        B = [
            C1;
            x1*C1;
            0; 
        ];
        states(1:3) = (states(1:3) + inv(inertia_matrix)*dt*(A*states + B*delta(i))); % inv(inertia_matrix)*(A_dis * states + B_dis * delta(i));
        states(4) = states(4) + dt*states(3);
        states_arr(:, i) = states;
    end
end


function test_model(dt, t, m, x1, x2, C1, C2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta)

    % Conversion factors:
    deg2rad = pi / 180;
    rad2deg = 180 / pi;
    in2ft = 1 / 12;
    ft2in = 12;
    mph2ftps = 5280 / 3600;
    ftps2mph = 3600 / 5280;

    % Ordinary bicycle model setup:
    Ca = C1 + C2;
    Cb = x1*C1 + x2*C2;
    Cc = x1*x1*C1 + x2*x2*C2;

    l2 = x1 - x2;

    C_phi1 = C1*eps1; C_phi2 = C2*eps2;
    
    % Calculate our understeering coefficient:
    K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
    K_roll_effect = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);
    
    K_understeer = K_understeer_wout_roll + K_roll_effect;


    figure;
    subplot(3,1,1);
    xlabel('Time (seconds)');
    ylabel('Yaw rate (rad/sec)');
    title(['Yaw rate response for ε1= ' num2str(eps1) ' and ε2= ' num2str(eps2) ' and Ku= ' num2str(K_understeer)]);
    hold on;

    subplot(3,1,2);
    xlabel('Time (seconds)');
    ylabel('Drift angle (rad)')
    title(['Drift angle response for ε1= ' num2str(eps1) ', ε2= ' num2str(eps2) ' and Ku= ' num2str(K_understeer)]);
    hold on;
    grid on;
    
    subplot(3,1,3);
    xlabel('Time (seconds)');
    ylabel('Roll (rad)');
    title(['Roll angle response for ε1= ' num2str(eps1) ', ε2= ' num2str(eps2) ' and Ku= ' num2str(K_understeer)]);
    hold on;
    
    legend_yaw_rate_arr = cell(1, length(speeds));
    legend_drift_angle_arr = cell(1, length(speeds));
    legend_roll_angle_arr = cell(1, length(speeds));


    % Simulate for different speeds:
    for i = 1:length(speeds)

        u = speeds(i);

        % Simulate:
        states_arr = simulate_bike_3dof(dt, t, m, x1, x2, C1, C2, Iz, u, delta, ms, h, K_phi, D_phi, eps1, eps2, Ix, c);
    
        subplot(3,1,1);
        plot(t, states_arr(2,:));
        legend_yaw_rate_arr{i} = ['Yaw rate @ ' num2str(u*ftps2mph) 'mph'];
        
        subplot(3,1,2);
        plot(t, states_arr(1,:) / u);
        legend_drift_angle_arr{i} = ['Drift angle @ ' num2str(u*ftps2mph) 'mph'];
    
        subplot(3,1,3);
        plot(t, states_arr(4,:));
        legend_roll_angle_arr{i} = ['Roll angle @ ' num2str(u*ftps2mph) 'mph'];

    end
    
    
    subplot(3,1,1);
    legend(legend_yaw_rate_arr);
    grid on;
    
    subplot(3,1,2);
    legend(legend_drift_angle_arr);
    grid on;

    subplot(3,1,3);
    legend(legend_roll_angle_arr);
    grid on;
    
    hold off;
    
end



function test_nonlinear_model(dt, t, m, x1, x2, Iz, speeds, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, delta, W)

    % Conversion factors:
    deg2rad = pi / 180;
    rad2deg = 180 / pi;
    in2ft = 1 / 12;
    ft2in = 12;
    mph2ftps = 5280 / 3600;
    ftps2mph = 3600 / 5280;

    % % Ordinary bicycle model setup:
    % Ca = C1 + C2;
    % Cb = x1*C1 + x2*C2;
    % Cc = x1*x1*C1 + x2*x2*C2;
    % 
    % l2 = x1 - x2;
    % 
    % C_phi1 = C1*eps1; C_phi2 = C2*eps2;
    
    % Calculate our understeering coefficient:
    % K_understeer_wout_roll = (-m*(Cb)/(C1*C2*l2));
    % K_roll_effect = (ms*h/K_phi)*(Cb*(C_phi1 + C_phi2) - Ca*(x1*C_phi1 + x2*C_phi2))/(C1*C2*l2);
    % 
    % K_understeer = K_understeer_wout_roll + K_roll_effect;


    figure(1);
    subplot(3,1,1);
    xlabel('Time (seconds)');
    ylabel('Yaw rate (rad/sec)');
    title(['Yaw rate response for ε1= ' num2str(eps1) ' and ε2= ' num2str(eps2)]);
    hold on;

    subplot(3,1,2);
    xlabel('Time (seconds)');
    ylabel('Drift angle (rad)')
    title(['Drift angle response for ε1= ' num2str(eps1) ', ε2= ' num2str(eps2)]);
    hold on;
    grid on;
    
    subplot(3,1,3);
    xlabel('Time (seconds)');
    ylabel('Roll (rad)');
    title(['Roll angle response for ε1= ' num2str(eps1) ', ε2= ' num2str(eps2)]);
    hold on;

    figure(2);
    subplot(4,1,1);
    xlabel('Time (seconds)');
    ylabel('Front left weight transfer (lbs)');
    title(['Left weight transfer with ε1= ' num2str(eps1) ' and ε2= ' num2str(eps2)]);
    hold on;

    subplot(4,1,2);
    xlabel('Time (seconds)');
    ylabel('Front right weight transfer (lbs)')
    title(['Front right weight transfer for ε1= ' num2str(eps1) ', ε2= ' num2str(eps2)]);
    hold on;
    grid on;
    
    subplot(4,1,3);
    xlabel('Time (seconds)');
    ylabel('Rear left weight transfer (lbs)');
    title(['Rear left weight transfer for ε1= ' num2str(eps1) ', ε2= ' num2str(eps2)]);
    hold on;

    subplot(4,1,4);
    xlabel('Time (seconds)');
    ylabel('Rear right weight transfer (lbs)')
    title(['Rear right weight transfer ε1= ' num2str(eps1) ', ε2= ' num2str(eps2)]);
    hold on;
    grid on;
    
    legend_yaw_rate_arr = cell(1, length(speeds));
    legend_drift_angle_arr = cell(1, length(speeds));
    legend_roll_angle_arr = cell(1, length(speeds));

    legend_W1l_arr = cell(1, length(speeds));
    legend_W1r_arr = cell(1, length(speeds));
    legend_W2l_arr = cell(1, length(speeds));
    legend_W2r_arr = cell(1, length(speeds));


    % Simulate for different speeds:
    for i = 1:length(speeds)

        u = speeds(i);

        % Simulate:
        [W1l_arr, W1r_arr, W2l_arr, W2r_arr, states_arr] = simulate_non_linear_bike_3dof(dt, t, m, x1, x2, Iz, u, delta, ms, h, K_phi, D_phi, eps1, eps2, Ix, c, W);
        
        % States time response plots:

        figure(1);
        subplot(3,1,1);
        plot(t, states_arr(2,:));
        legend_yaw_rate_arr{i} = ['Yaw rate @ ' num2str(u*ftps2mph) 'mph'];
        
        subplot(3,1,2);
        plot(t, states_arr(1,:) / u);
        legend_drift_angle_arr{i} = ['Drift angle @ ' num2str(u*ftps2mph) 'mph'];
    
        subplot(3,1,3);
        plot(t, states_arr(4,:));
        legend_roll_angle_arr{i} = ['Roll angle @ ' num2str(u*ftps2mph) 'mph'];


        % Weight transfer time response plots:

        figure(2);
        subplot(4,1,1);
        plot(t, W1l_arr);
        legend_W1l_arr{i} = ['Front left weight transfer @ ' num2str(u*ftps2mph) 'mph'];
        
        subplot(4,1,2);
        plot(t, W1r_arr);
        legend_W1r_arr{i} = ['Front right weight transfer @ ' num2str(u*ftps2mph) 'mph'];
    
        subplot(4,1,3);
        plot(t, W2l_arr);
        legend_W2l_arr{i} = ['Rear left weight transfer @ ' num2str(u*ftps2mph) 'mph'];

        subplot(4,1,4);
        plot(t, W2r_arr);
        legend_W2r_arr{i} = ['Rear right weight transfer @ ' num2str(u*ftps2mph) 'mph'];

    end
    
    % First figure:
    figure(1);
    subplot(3,1,1);
    legend(legend_yaw_rate_arr);
    grid on;
    hold off;
    
    subplot(3,1,2);
    legend(legend_drift_angle_arr);
    grid on;
    hold off;

    subplot(3,1,3);
    legend(legend_roll_angle_arr);
    grid on;
    hold off;
    
    % Second figure:
    figure(2);
    subplot(4,1,1);
    legend(legend_W1l_arr);
    grid on;
    hold off;
    
    subplot(4,1,2);
    legend(legend_W1r_arr);
    grid on;
    hold off;

    subplot(4,1,3);
    legend(legend_W2l_arr);
    grid on;
    hold off;

    subplot(4,1,4);
    legend(legend_W2r_arr);
    grid on;
    hold off;
    
end

function delta = slope_it_down(t, dt, at, delta, left_bound, right_bound, slope)

    time_to_reach = (right_bound - left_bound) / slope; % time units
    delta(1, at/dt:(at + time_to_reach)/dt) = left_bound + slope * (t(at/dt:(at + time_to_reach)/dt) - t(at/dt)); % * sign(right_bound - left_bound);

end








