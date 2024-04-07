classdef LateralController < handle
    properties
        State KalmanFilter
        Vp VehicleParameters
        ControlFn
        Planner
        LogFn = @(x)(0)
        DeviationDisturbFn = @(~, x) x
        InputDisturbFn = @(~, x) x
        SenseIntervalTicks int64 {mustBeGreaterThan(SenseIntervalTicks, 0)} = 1
        PlanningIntervalTicks int64 {mustBeGreaterThan(PlanningIntervalTicks, 0)} = 1
        ReferencePath
    end

    properties(Access=private)
        TickCounter int64 = 0
        FrenetTrajectory = missing
    end
    
    methods
        function [u] = tick(obj, t, interval, simulation_x, vx)            
            % Compute A,B for this velocity
            sys = bicycle_lat_roadalign_factory(obj.Vp, interval, vx);
            obj.State.A = sys.A;
            obj.State.B = sys.B;

            % Update (w/ KF)
            z = obj.ReferencePath.deviation(simulation_x(1:2), simulation_x(4));
            z = obj.DeviationDisturbFn(t, z);
            if (mod(obj.TickCounter, obj.SenseIntervalTicks) == 0)
                obj.State.update(z);
            end

            % Query the planner
            s = obj.ReferencePath.arc(simulation_x(1:2));
            
            xdes = missing;
            if (mod(obj.TickCounter, obj.PlanningIntervalTicks) == 0)
                % When updating the trajectory, get the xdes for the old
                % one. Otherwise, xdes will be equal to obj.State.X!
                if ~ismissing(obj.FrenetTrajectory)
                    xdes = ppval(obj.FrenetTrajectory, s);
                end
                obj.FrenetTrajectory = obj.Planner.get_path(obj.State.X, s, sys, vx);
            end

            if ismissing(xdes)
                % If we did not update the trajectory, or if the trajectory
                % did not exist.
                xdes = ppval(obj.FrenetTrajectory, s);
            end

            % Control (w/ PP/LQR)
            u = obj.ControlFn(obj.State.X, xdes, obj.State.A, obj.State.B);

            % Log
            dev = [obj.State.X(1); obj.State.X(3)];
            obj.LogFn([
                t;
                dev; % deviation state
                z; % deviation measurement
                dev - z; % error
                obj.State.X(2); % Lateral Deviation Rate (m/s)
                obj.State.X(4); % Heading Deviation Rate (rad/s)
                u; % Commanded Steering Input
                obj.ReferencePath.position(s) + obj.ReferencePath.normal(s) * xdes(1);
                vx;
                obj.ReferencePath.angle(s) + xdes(3);
            ]);
            
            % Note that here we assume that we know our exact position
            % along the path. This allows us to avoid having to
            % design an estimator for the vehicle position.
            % I don't think it makes enough effect to be worth designing
            % and it doesn't make sense to add noise to the radius.
            angveldes = vx * obj.ReferencePath.curvature(s);

            % Predict (w/ ss eqn.)
            obj.State.predict([
                u;
                angveldes
            ]);
            
            % Disturb the actual input
            % u = obj.InputDisturbFn(t, u);
            
            obj.TickCounter = obj.TickCounter + 1;
        end
    end
end

