classdef RHPlanner < Planner
    properties
        Obstacles (1,:) cell
        SafetyDistance (1,1) double
        UMax (1,1) double
        Horizon (1,1) int64
        StepDT (1,1) double
        ReferencePath
        RefPathBoundsPP struct
        LogFn
        TargetLateralDevFn = @(s) repelem(0, numel(s), 1)
    end

    methods
        function path = get_path(obj, x0, s0, sys, vx)
            ds = obj.StepDT * vx;
            s = s0 + double(1:obj.Horizon) * ds;
            tgt_ld = obj.TargetLateralDevFn(s);

            rpath_positions = obj.ReferencePath.position(s);
            rpath_normals = obj.ReferencePath.normal(s);
            rpath_phidotdes = vx * obj.ReferencePath.curvature(s);
            rpath_bounds = ppval(obj.RefPathBoundsPP, s);

            optimizer = optimizer_factory(sys, obj.UMax, obj.Horizon);
            frenet_obs = frenet_obstacles(obj.Obstacles, rpath_positions, rpath_normals, obj.SafetyDistance);
            bounds = boundgen(frenet_obs, rpath_bounds(1,:), rpath_bounds(2,:));
            paths = bounds_to_paths(bounds);

            [us, ~] = best_path_planner(optimizer, x0, tgt_ld, rpath_phidotdes, paths);
            if ~ismissing(us)
                frenet_traj = input2frenet(sys.A, sys.B, x0, rpath_phidotdes, us);
                path = spline([s0, s], [x0, frenet_traj]);
            else
                path = spline([0, 1], [x0, x0]);
            end
        end
    end
end