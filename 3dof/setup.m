mass = 0.5 %kg
mdot = poly(1); %polyfit fuel mdot




motor = table2array(readtable('motor.txt'));

thrust_poly = poly(1); %polyfit thrust


earth_radius = 6378e3; %m 
earth_mass = 5.9722 * 10 ^ (24); %kg
G = 6.6743 * 10 ^ (-11); 


save("params.mat");