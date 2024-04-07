classdef ConvexObstacle
    methods (Abstract)
        [tmin, tmax] = line_intersection(origin, tangent);
        [sd] = signed_distance(point);
    end
end