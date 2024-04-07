classdef AABBObstacle < ConvexObstacle
    properties
        Center (2,1)
        Size (2,1)
    end
    
    methods
        function obj = AABBObstacle(center, size)
            obj.Center = center;
            obj.Size = size;
        end

        function [tminmax] = line_intersection(obj, origin, tangent)
            size_2 = obj.Size / 2;

            bl = obj.Center - size_2; % bottom left corner
            tr = obj.Center + size_2; % top right corner
            
            % Normalize tangent to use distance as line parameter
            tangent = tangent / norm(tangent);

            % Test intersection with each segment
            tus = [
                line_line_intersection(origin, tangent, bl, [ 1; 0] .* obj.Size), ...
                line_line_intersection(origin, tangent, bl, [ 0; 1] .* obj.Size), ...
                line_line_intersection(origin, tangent, tr, [-1; 0] .* obj.Size), ...
                line_line_intersection(origin, tangent, tr, [ 0;-1] .* obj.Size)
            ];
            
            % Hit when u (segment parameter) is in [0,1]
            hits = tus(2,:) >= 0-eps & tus(2,:) <= 1+eps;

            if sum(hits) == 0
                tminmax = [NaN, NaN];
                return;
            end
            
            % Retrieve min and max line parameters
            ts = tus(1,hits);
            tminmax = mbounds(ts);
        end

        function [sd] = signed_distance(obj, pt)
            pt = pt - obj.Center;
            size_2 = obj.Size / 2;
            
            fq = abs(pt); % transform to first quadrant (simplify by symmetry)
            trv = fq - size_2; % vector from top right vertex
            
            outer = vecnorm(max(trv, 0)); % if any(trv > 0) then we're outside the box
            inner = min(max(trv), 0); % if any(trv < 0) then we're inside the box
            sd = outer + inner; % only one can be != 0
        end
    end
end

