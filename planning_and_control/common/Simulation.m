classdef Simulation < handle
    properties
        Vp VehicleParameters = VehicleParameters
        InitialLongVel = 70 / 3.6 % [m/s]
        TMax double = 30 % [s]
        CtrlInterval double = 1/20 % [s]
        
        GlobalInitialPosition (2,1) double = [0;0]
        PlanningIntervalTicks int32 = 50 % [ctrl ticks]
        SenseIntervalTicks int32 = 1 % [ctrl ticks]
        Path {mustBeA(Path, {'missing', 'ReferencePath'})} = missing
        Obstacles (1,:) cell = {}
        LatEstX0 (4,1) = zeros(4,1)
        LatEstSigma0 double = 1e-3
        LatEstMeasureSigma double = 1e-3
        LatEstProcessSigma double = 1e-3
        LateralControlFn {mustBeA(LateralControlFn, {'missing', 'function_handle'})} = missing
        LatInputDisturbFn {mustBeA(LatInputDisturbFn, {'missing', 'function_handle'})} = missing
        LatDeviationDisturbFn {mustBeA(LatDeviationDisturbFn, {'missing', 'function_handle'})} = missing
        TargetLatDeviationFn {mustBeA(TargetLatDeviationFn, {'missing', 'function_handle'})} = missing
        
        SafetyDistance double = 0.5 % [m]
        PlanningHorizon int64 = 70 % [ctrl ticks]
        PathBoundsPP struct = spline([0,1], [-3.5,-3.5;3.5,3.5]) % s -> [m], default to point 3

        TgtLongVelFn = @(~, ~) 70 / 3.6 % [m/s]
        LongKp double = 100
        LongKi double = 50
        LogStep = 0.01
    end

    methods
        function [lat_sim_data, long_sim_data, lat_ctrl_data, long_ctrl_data, planning_data] = run(obj)
            %% Simulation parameters
            x0 = [
                obj.InitialLongVel / obj.Vp.WheelRadius; % Wheel angular velocity (rad/s)
                obj.InitialLongVel; % Longitudinal Velocity (m/s)
                obj.GlobalInitialPosition(1); % X(m)
                obj.GlobalInitialPosition(2); % Y(m)
                0; % Lateral Velocity(m/s)
                0; % Heading(rad)
                0; % Yaw Rate(rad/s)
            ];
            lat_sim_log = Logger(obj.LogStep);
            long_sim_log = Logger(obj.LogStep);

            xdot_fn = full_xdot_factory(obj.Vp, @lat_sim_log.log, @long_sim_log.log);
            
            %% Control parameters
            if ismissing(obj.Path)
                % Default to point 1
                obj.Path = MultiReferencePath({
                    LineReferencePath([0; 0], [1; 0]);
                    CircleReferencePath([200; 1000], 1000, -pi/2);
                    LineReferencePath([1200; 1000], [0;1])
                }, [200, 1000 * pi/2]);
            end

            assert(all(cellfun(@(x) isa(x, 'ConvexObstacle'), obj.Obstacles), 'all'), ...
                'Obstacles must be a (possibly empty) cell vector of ConvexObstacle(s)' ...
            );

            if ismissing(obj.LateralControlFn)
                % Default to LQR control
                Q = [1, 0, 0, 0;
                     0, 0.3, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0];
                R = 1;
                lat_ctrl_fn = lqr_ctrl_factory(obj.Vp, Q, R, obj.CtrlInterval);
            else
                lat_ctrl_fn = obj.LateralControlFn;
            end

            lat_ctrl_log = Logger(obj.LogStep);
            lat_ctrl = LateralController;
            lat_ctrl.State = kalman_factory( ...
                obj.LatEstX0, ...
                obj.LatEstSigma0, ...
                obj.LatEstMeasureSigma, ...
                obj.LatEstProcessSigma ...
            );
            lat_ctrl.LogFn = @lat_ctrl_log.log;

            planner_log = Logger;
            lat_ctrl.ReferencePath = obj.Path;
            lat_ctrl.Planner = RHPlanner;
            lat_ctrl.Planner.LogFn = @planner_log.log;
            lat_ctrl.Planner.Obstacles = obj.Obstacles;
            lat_ctrl.Planner.SafetyDistance = obj.SafetyDistance;
            lat_ctrl.Planner.UMax = obj.Vp.MaxSteer;
            lat_ctrl.Planner.Horizon = obj.PlanningHorizon;
            lat_ctrl.Planner.StepDT = obj.CtrlInterval;
            lat_ctrl.Planner.ReferencePath = obj.Path;
            lat_ctrl.Planner.RefPathBoundsPP = obj.PathBoundsPP;
            if ~ismissing(obj.TargetLatDeviationFn)
                lat_ctrl.Planner.TargetLateralDevFn = obj.TargetLatDeviationFn;
            end
            lat_ctrl.SenseIntervalTicks = obj.SenseIntervalTicks;
            lat_ctrl.PlanningIntervalTicks = obj.PlanningIntervalTicks;
            lat_ctrl.Vp = obj.Vp;
            lat_ctrl.ControlFn = lat_ctrl_fn;
            if ~ismissing(obj.LatInputDisturbFn)
                lat_ctrl.InputDisturbFn = obj.LatInputDisturbFn;
            end
            if ~ismissing(obj.LatDeviationDisturbFn)
                lat_ctrl.DeviationDisturbFn = obj.LatDeviationDisturbFn;
            end

            long_ctrl_log = Logger(obj.LogStep);
            long_ctrl = LongitudinalPIController;
            long_ctrl.LogFn = @long_ctrl_log.log;
            long_ctrl.Kp = obj.LongKp;
            long_ctrl.Ki = obj.LongKi;
            long_ctrl.TargetVelocityFn = obj.TgtLongVelFn;

            input_fn = full_ctrl_factory(@lat_ctrl.tick, @long_ctrl.tick);
            
            %% Simulate
            discrete_ctrl_sim(x0, 0, obj.TMax, obj.CtrlInterval, xdot_fn, input_fn);

            lat_sim_data = lat_sim_log.retrieve();
            long_sim_data = long_sim_log.retrieve();
            lat_ctrl_data = lat_ctrl_log.retrieve();
            long_ctrl_data = long_ctrl_log.retrieve();
            planning_data = planner_log.retrieve();
        end
    end
end



