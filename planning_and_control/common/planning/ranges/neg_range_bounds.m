function [neg_bounds] = neg_range_bounds(bounds_in, dx)
    arguments
        bounds_in (1,:) double
        dx (1,1) double = eps
    end

    neg_bounds = remove_near_bounds([-Inf, bounds_in, Inf], dx);
end