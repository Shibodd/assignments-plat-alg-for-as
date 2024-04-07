classdef NullPath < ReferencePath
    methods
        function s = arc(obj, position)
            s = 0;
        end
        function z = deviation(obj, position, heading)
            z = [0;0];
        end
        
        function position = position(obj, s)
            position = [0;0];
        end
        function k = curvature(obj, s)
            k = 0;
        end
        function theta = angle(obj, s)
            theta = 0;
        end
        function normal = normal(obj, s)
            normal = [0;0];
        end
    end
end

