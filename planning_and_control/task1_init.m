clear;
addpath(genpath("assignment3"));

sim = Simulation;
sim.InitialLongVel = 90 / 3.6; % [km/h]
sim.TgtLongVelFn = @(~,~) sim.InitialLongVel; % Constant velocity
sim.Path = turn90deg_factory(200, 1000); % 200m followed by a 1000m radius circle

tic;
[lat_sim_data, long_sim_data, lat_ctrl_data, long_ctrl_data, planning_data] = sim.run();
toc;