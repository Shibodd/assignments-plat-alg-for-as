clear;
addpath(genpath("assignment3"));

LW = 3.5; % Lane Width [m]
obs1_s = 100; % Obstacle 1 position, as reference path arc length [m]
obs2_s = 550; % Obstacle 2 position, as reference path arc length [m]

sim = Simulation;
sim.Path = turn90deg_factory(200, 1000); % 200m straight, followed by 1000m radius circle
sim.InitialLongVel = 50 / 3.6; % [km/h]
sim.TgtLongVelFn = @(~,~) sim.InitialLongVel; % Constant velocity
sim.PathBoundsPP = spline([0, 1], repmat([-LW;LW],1,2)); % min; max - constant
sim.TMax = 60;
sim.SafetyDistance = 0.5 + 1.86/2; % Safety distance + car width [m]
sim.TargetLatDeviationFn = @(s) repelem(-LW/2, numel(s), 1); % target lateral deviation - right lane, constant
sim.GlobalInitialPosition = [0;-LW/2]; % Initial actual position of vehicle
sim.LatEstX0 = [-LW/2;0;0;0]; % Initial estimate of state in frenet space


sim.Obstacles = {
    AABBObstacle(sim.Path.position(obs1_s) - sim.Path.normal(obs1_s)*LW/2, [LW,LW]);
    AABBObstacle(sim.Path.position(obs2_s) - sim.Path.normal(obs2_s)*LW/2, [LW,LW])
};

tic;
[lat_sim_data, long_sim_data, lat_ctrl_data, long_ctrl_data, planning_data] = sim.run();
toc;