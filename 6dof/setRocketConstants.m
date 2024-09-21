clc;
clear;



%initial state
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0]; %[x; y; z; vx; vy; vz; ax; ay; az]
q0 = [1; 0; 0; 0];

%rocketconfig   
wetmass = 0.3; %kg
drymass = 0.25;
currentmass = 0.3;

diameter = 0.4;
S = (pi/4)*0.4^2;


motor = table2array(readtable('motor.txt'));


%GNC
qBody2IMU = [1 0 0 0];
time_step = 0.01;  % consider running sim and 'controls' at different hz??

%motorcurve = interp1(motor(:, 1), motor(:, 2), 0:0.01:2.25)
%plot(0:0.01:2.25, motorcurve)
%motorcurve = interp1(motor(:, 1), motor(:, 2), 2.43)




%constants

earth_radius = 6371852; %m ASSUMING LAUNCH IS AT SPACEPORT
earth_mass = 5.9722 * 10 ^ (24); %kg
G = 6.6743 * 10 ^ (-11); 


save("params.mat");

%{
hws = get_param(bdroot, 'modelworkspace');
hws.DataSource = 'MAT-File';
hws.FileName = 'params';

data = dictionary("mass", 0.5, "mmoi", 0.1);

keys = keys(data);
for i = 1:length(keys)
    hws.assignin(keys(i), data(keys(i)));
end

%}
