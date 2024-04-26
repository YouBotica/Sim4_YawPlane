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
ylim([0, 10.0]);

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
    % 
    % K_understeer = K_understeer_wout_roll + K_roll_effect;
    % 
    % delta2r_gain_arr(i) = (u) / (l2 + u*u*K_understeer);
    
    delta2r_gain_arr(i) = (u*C1*(Cb-x1*Ca)) /(Cb*Cb - Ca*Cc + Cb*m*u*u + (x1*Ca*C_phi1 + x2*Ca*C_phi2 - (C_phi1 + C_phi2)*Cb)*(ms*h*u*u / K_phi));

    % plot each gain in an xy plot:
    plot(t, delta2r_gain_arr(i)*ones(length(t)), '--', 'LineWidth', 2);
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

    subplot(3,1,1);
    plot(t, delta2r_gain_arr(i)*ones(length(t)), '--', 'LineWidth', 2);
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
legend(legend_gains_arr);

subplot(3,1,2);
legend(legend_yaw_rate_arr);

subplot(3,1,3);
legend(legend_steering_angle_arr);

hold off;






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
        (C1)/(m*u);
        (x1*C1)/Iz;
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
        states(1:3) = states(1:3) + dt*(inv(inertia_matrix)*(A*states + B*delta(i))); % inv(inertia_matrix)*(A_dis * states + B_dis * delta(i));
        states(4) = states(4) + dt*states(3);
        states_arr(:, i) = states;
    end
end


function delta = slope_it_down(t, dt, at, delta, left_bound, right_bound, slope)

    time_to_reach = (right_bound - left_bound) / slope; % time units
    delta(1, at/dt:(at + time_to_reach)/dt) = left_bound + slope * (t(at/dt:(at + time_to_reach)/dt) - t(at/dt)); % * sign(right_bound - left_bound);

end








