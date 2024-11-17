clc;
clear all;
close all;

% Parameters
l = 2.54;  % Wheelbase (m)
a = 1.14;  % Distance from CG to front axle (m)
b = l - a; % Distance from CG to rear axle (m)
m = 1000;  % Vehicle mass (kg)
Iz = 1100; % Yaw moment of inertia (kg.m^2)
Caf = 2400 * (180 / pi); % Front cornering stiffness (N/rad)
Car = 2050 * (180 / pi); % Rear cornering stiffness (N/rad)

% Set the steering angle input (delta_f)
delta_f = 1 * (pi / 180); % Convert 1 degree to radians

% Range of forward velocities
u_range = 5:1:40;  % Forward velocity range from 5 to 40 m/s

% Initialize arrays to store results
yaw_rate_gains = zeros(size(u_range));
lateral_acceleration_gains = zeros(size(u_range));

for i = 1:length(u_range)
    u = u_range(i);
    
    % State-space matrices
    A = [-(Caf+Car)/(m * u), ((b*Car-a*Caf)/(m*u))-u ;
          (b*Car-a*Caf)/(u*Iz), -(a^2 * Caf + b^2 * Car)/(Iz * u)];

    B = [Caf/m, Car/m;
        (a*Caf)/Iz, (-b* Car)/Iz];

    % Solve for the steady-state response
    steady_state = -A \ (B * delta_f);

    % Extract steady-state yaw rate (r) and lateral acceleration (a_y)
    r = steady_state(2); % Steady-state yaw rate
    a_y = steady_state(1); % Steady-state lateral acceleration

    % Convert yaw rate to (deg/sec)/deg
    yaw_rate_gains(i) = r / delta_f * (180/pi); % (deg/s)/deg

    % Convert lateral acceleration to (m/s^2)/deg
    lateral_acceleration_gains(i) = a_y / delta_f * (180/pi); % (m/s^2)/deg
end

% Plotting the results
figure;
subplot(2,1,1);
plot(u_range, yaw_rate_gains, 'b', 'LineWidth', 2);
title('Steady-State Yaw Rate Gain vs Forward Velocity');
xlabel('Forward Velocity (m/s)');
ylabel('Yaw Rate Gain (deg/s)/deg');
grid on;

subplot(2,1,2);
plot(u_range, lateral_acceleration_gains, 'r', 'LineWidth', 2);
title('Steady-State Lateral Acceleration Gain vs Forward Velocity');
xlabel('Forward Velocity (m/s)');
ylabel('Lateral Acceleration Gain (m/s^2)/deg');
grid on;
