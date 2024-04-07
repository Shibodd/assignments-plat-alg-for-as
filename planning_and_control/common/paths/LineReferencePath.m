classdef LineReferencePath < ReferencePath
    properties(Access=private)
        Tangent (2,1) double
        Origin (2,1) double

        % Cached derived values
        Normal (2,1) double
        Heading (1,1) double
    end
    
    methods
        function obj = LineReferencePath(origin, tangent)
            arguments
                origin (2,1) double
                tangent (2,1) double
            end

            obj.Origin = origin;
            obj.Tangent = tangent / norm(tangent);

            % Derived values
            obj.Normal = [-obj.Tangent(2); obj.Tangent(1)];
            obj.Heading = atan2(obj.Tangent(2), obj.Tangent(1));
        end

        function arc = arc(obj, position)
            arc = dot(obj.Tangent, position - obj.Origin);
        end
        
        function z = deviation(obj, position, heading)
            centerline = obj.position(obj.arc(position));
            ld = dot(obj.Normal, position - centerline); % Signed distance to centerline

            z = [
                ld;
                heading - obj.Heading
            ];
        end

        function position = position(obj, s)
            position = obj.Origin + obj.Tangent * s;
        end

        function k = curvature(~, s)
            k = repelem(0, 1, numel(s));
        end

        function theta = angle(obj, s)
            theta = repelem(obj.Heading, 1, numel(s));
        end

        function normal = normal(obj, s)
            normal = repmat(obj.Normal, 1, numel(s));
        end
    end
end

