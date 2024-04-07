classdef FrenetReferencePath < ReferencePath
    properties(Access=private)
        RefPath (1,1)
        FrenetTrajectory (4,:) double
        TrajS (1,:) double
        Deriv1 (1,1)
        Deriv2 (1,1)
    end

    methods(Access=private)
        function x = interpolate_frenet_traj(obj, s)
            x = interp1(obj.TrajS, obj.FrenetTrajectory', s)';
            x(isnan(x)) = 0;
            % assert(~any(isnan(x),"all"), "FIXME, interp1 out of bounds");
        end
    end
    
    methods
        function obj = FrenetReferencePath(refpath, traj_frenet, traj_s, ~)
            obj.RefPath = refpath;
            obj.FrenetTrajectory = traj_frenet;
            obj.TrajS = traj_s;
            
            xy = obj.position(traj_s);
            spl = spline(traj_s, xy);
            obj.Deriv1 = fnder(spl, 1);
            obj.Deriv2 = fnder(spl, 2);
        end

        function s = arc(obj, position)
            s = obj.RefPath.arc(position);
        end

        function z = deviation(obj, position, heading)
            z = obj.RefPath.deviation(position, heading);

            s = obj.RefPath.arc(position);
            x = obj.interpolate_frenet_traj(s);

            z = z - [x(1);x(3)];
        end

        function position = position(obj, s)
            x = obj.interpolate_frenet_traj(s);

            position = obj.RefPath.position(s);
            position = position + obj.RefPath.normal(s) .* x(1,:);
        end

        function k = curvature(obj, s)
            d = ppval(obj.Deriv1, s);
            d2 = ppval(obj.Deriv2, s);
            
            num = (d(1) .* d2(2) - d(2) .* d2(1));
            den = (d(1).^2 + d(2).^2) .^ (1.5);
            
            % Compute the curvature
            k = num ./ den;
        end

        function theta = angle(obj, s)
            x = obj.interpolate_frenet_traj(s);
            theta = x(3) + obj.RefPath.angle(s);
        end

        function normal = normal(~, ~)
            assert(false, "NOT IMPLEMENTED");
            normal = [];
        end
    end
end