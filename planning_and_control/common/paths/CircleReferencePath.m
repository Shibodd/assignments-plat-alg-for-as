classdef CircleReferencePath < ReferencePath
    properties(Access=protected)
        Center (2,1) double
        Radius double
        Direction {mustBeMember(Direction, [1, -1])} = 1
        Phi0 double
    end
    
    methods
        function phi = phi_at(obj, s)
            phi = obj.Phi0 + (obj.Direction * s / obj.Radius);
        end
    end

    methods
        function obj = CircleReferencePath(center, radius, phi0, direction)
            arguments
                center (2,1) double
                radius (1,1) double
                phi0 (1,1) double
                direction {mustBeMember(direction, [1, -1])} = 1
            end

            obj.Center = center;
            obj.Radius = radius;
            obj.Phi0 = phi0;
            obj.Direction = direction;
        end

        function s = arc(obj, position)
            r = position - obj.Center;
            phi = atan2(r(2), r(1));
            s = mod(phi - obj.Phi0, 2*pi) * obj.Radius;
        end
        
        function z = deviation(obj, position, heading)
            r = position - obj.Center;
            ld = (obj.Radius - norm(r)) * obj.Direction;

            a = heading - obj.angle(obj.arc(position));
            a = mod(a + pi, 2*pi) - pi;
    
            z = [
                ld;
                a
            ];
        end

        function v = normal(obj, s)
            phi = obj.phi_at(s);
            v = -[cos(phi); sin(phi)];
        end

        function pt = position(obj, s)
            pt = obj.Center - obj.normal(s) * obj.Radius;
        end

        function theta = angle(obj, s)
            theta = obj.phi_at(s) + obj.Direction * pi/2;
        end

        function k = curvature(obj, s)
            k = repelem(1 / obj.Radius, numel(s));
        end
    end
end
