data = readmatrix('i55_thrust.csv');  

% Extract time and thrust columns
t = data(:,1);  
T = data(:,2);  

t_interp = linspace(0, 7.3, 100);  

T_pchip = interp1(t, T, t_interp, 'pchip'); 

% Plot results
figure;
plot(t, T, 'o', 'MarkerSize', 10, 'DisplayName', 'Data points'); hold on;
plot(t_interp, T_pchip, '--', 'DisplayName', 'PCHIP Interp');
xlabel('Time (s)');
ylabel('Thrust (N)');
grid on;
title('i55 Thrust Curve');

initial_weight = 3000;
mass_flow_rate = 223.8/7.2; %propellant mass/burn time
time = 0:0.1:7.3; 

mass = initial_weight - mass_flow_rate * time;

% Plot the rocket mass over time
figure;
plot(time, mass, '-','LineWidth', 2);
title('Time vs Rocket Mass');
xlabel('Time (seconds)');
ylabel('Mass (kg)');
grid on;