tiledlayout(4,2);

nexttile([1, 2]);
plot(lat_sim_data(2, :), lat_sim_data(3, :)); % position
axis equal;
title('Vehicle Trajectory');
xlabel('X[m]');
ylabel('Y[m]');

% Obstacles
for i=1:numel(sim.Obstacles)
    obs = sim.Obstacles{i};
    rectangle('Position', [obs.Center - obs.Size / 2; obs.Size]');
end

s0 = sim.Path.arc(lat_sim_data(2:3,1));
sN = sim.Path.arc(lat_sim_data(2:3,end));
s = s0:1:sN;
path_pos = sim.Path.position(s);
path_normals = sim.Path.normal(s);

lane_boundaries = ppval(sim.PathBoundsPP, s);
minbound = path_pos + path_normals .* lane_boundaries(1,:);
maxbound = path_pos + path_normals .* lane_boundaries(2,:);

line(minbound(1,:), minbound(2,:), 'LineStyle', '--');
line(maxbound(1,:), maxbound(2,:), 'LineStyle', '--');
line(path_pos(1,:), path_pos(2,:), 'LineStyle', '--');
line(NaN,NaN,'Color','Black');

legend({"Trajectory", "Left Bound", "Right Bound", "Reference Path", "Obstacles"});


t = lat_sim_data(1,:);
nexttile; plot(t, lat_sim_data(5, :)) % heading
title('Vehicle Heading');
xlabel('Time[s]');
ylabel('[rad]');

nexttile; plot(t, lat_sim_data(7, :)) % slip
title('Slip Angle');
xlabel('Time[s]');
ylabel('[rad]');

nexttile;
hold on;
plot(lat_ctrl_data(1,:), lat_ctrl_data(10,:)) % steering
title('Steering Angle');
xlabel('Time[s]');
ylabel('[rad]');

nexttile; plot(lat_ctrl_data(1,:), lat_ctrl_data(4,:)); % Lateral Deviation
title('Lateral Deviation');
xlabel('Time[s]');
ylabel('Deviation[m]');

clear t;

t = long_sim_data(1,:);
nexttile; plot(t, long_sim_data(3, :)) % velocity
title('Velocity');
xlabel('Time[s]');
ylabel('[m/s]');

nexttile; plot(t, long_sim_data(6, :)) % commanded torque (for each wheel)
title('Commanded Torque (same for each wheel)');
xlabel('Time[s]');
ylabel('[Nm]');
clear t;