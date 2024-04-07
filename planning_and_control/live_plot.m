addpath(genpath("assignment3/common/simviz"));

sv = SimViz;

tiledlayout(6,2);
nexttile([1, 2]);
sv.addSignal(lat_sim_data(1,:), lat_sim_data(2,:), lat_sim_data(3,:));
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(11,:), lat_ctrl_data(12,:));
axis equal;
legend('', 'Actual', '', 'Target');
title('Global Vehicle Position');
xlabel('X[m]');
ylabel('Y[m]');


nexttile([1, 1]);
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(4,:));
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(2,:));
legend('', 'Measured', '', 'Estimated');
title('Lateral Deviation');
xlabel('Time([s]');
ylabel('Deviation[m]');


nexttile([1, 1]);
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(5,:));
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(3,:));
legend('', 'Measured', '', 'Estimated');
title('Heading Deviation');
xlabel('Time[s]');
ylabel('Deviation[rad]');


nexttile([1, 1]);
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(8,:));
title('Lateral Deviation Rate');
xlabel('Time([s]');
ylabel('Deviation Rate[m/s]');


nexttile([1, 1]);
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(9,:));
title('Heading Deviation Rate');
xlabel('Time([s]');
ylabel('Deviation Rate[rad/s]');


nexttile([1, 1]);
sv.addSignal(lat_ctrl_data(1,:), lat_ctrl_data(1,:), lat_ctrl_data(10,:));
sv.addSignal(lat_sim_data(1,:), lat_sim_data(1,:), lat_sim_data(9,:));
legend('', 'Commanded', '', 'Actual (w/disturb)');
title('Steering Angle');
xlabel('Time([s]');
ylabel('Angle[rad]');


nexttile([1, 1]);
sv.addSignal(long_ctrl_data(1,:), long_ctrl_data(1,:), long_ctrl_data(4,:));
title('Commanded Torque');
xlabel('Time([s])');
ylabel('Torque[N]');


nexttile([1, 1]);
sv.addSignal(long_ctrl_data(1,:), long_ctrl_data(1,:), long_ctrl_data(6,:));
sv.addSignal(long_ctrl_data(1,:), long_ctrl_data(1,:), long_ctrl_data(3,:));
title('Longitudinal Velocity');
legend('', 'Actual', '', 'Target');
xlabel('Time([s])');
ylabel('Velocity[m/s]');


nexttile([1, 1]);
sv.addSignal(long_ctrl_data(1,:), long_ctrl_data(1,:), long_ctrl_data(2,:));
title('Longitudinal Velocity Error');
xlabel('Time([s])');
ylabel('Velocity[m/s]');


nexttile([1, 1]);
sv.addSignal(long_ctrl_data(1,:), long_ctrl_data(1,:), long_ctrl_data(end,:));
title('Integral');
xlabel('Time([s])');
ylabel('Value');

pause(1);
sv.run(1);