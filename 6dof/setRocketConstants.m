clear;



%initial state
x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0]; %[x; y; z; vx; vy; vz; ax; ay; az]
q0 = [1; 0; 0; 0];

%rocketconfig   TEMP
mass = 0.5; %kg
mmoi = 0.1;





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
