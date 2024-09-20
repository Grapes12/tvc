clc;
clear;



%initial state
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0]; %[x; y; z; vx; vy; vz; ax; ay; az]
q0 = [1; 0; 0; 0];

%rocketconfig   
wetmass = 0.3; %kg
drymass = 0.25;
currentmass = 0.3;

motor = table2array(readtable('motor.txt'));

%motorcurve = interp1(motor(:, 1), motor(:, 2), 0:0.01:2.25)
%plot(0:0.01:2.25, motorcurve)
%motorcurve = interp1(motor(:, 1), motor(:, 2), 2.43)

save("params.mat")

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
