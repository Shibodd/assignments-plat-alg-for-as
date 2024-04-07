clear;
addpath(genpath("assignment3"));

sim = Simulation;
sim.InitialLongVel = 30 / 3.6; % [km/h]
sim.TgtLongVelFn = @(~,~) sim.InitialLongVel; % Constant velocity
sim.Path = turn90deg_factory(20, 12); % 20m straight followed by a 12m radius circle
sim.PathBoundsPP = spline([0, 1], repmat([-1.5;1.5],1,2)); % min; max - constant
sim.TMax = 8;

tic;
[lat_sim_data, long_sim_data, lat_ctrl_data, long_ctrl_data, planning_data] = sim.run();
toc;