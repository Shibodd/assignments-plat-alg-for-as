classdef ReferencePath < handle
    methods(Abstract)
        s = arc(obj, position);
        z = deviation(obj, position, heading);
        
        position = position(obj, s);
        k = curvature(obj, s);
        theta = angle(obj, s);
        normal = normal(obj, s);
    end

    methods
        function ld = lateral_deviation(obj, position)
            z = obj.deviation(position, 0);
            ld = z(1);
        end
    end
end