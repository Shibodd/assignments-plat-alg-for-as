function [bounds_out] = remove_near_bounds(bounds_in, dx)
    % Remove near boundaries (apply threshold dx)

    arguments
        bounds_in (1,:) double
        dx (1,1) double = eps
    end
    
    % We use Infinity to denote an unbounded range - however,
    % Infinity - Infinity = NaN, so by only checking the diff we wouldn't
    % delete repeated Infinity values. Therefore, also check by equality.
    near_pairs = diff(bounds_in) <= dx;
    equal_pairs = bounds_in(1:end-1) == bounds_in(2:end);
    pair_nmask = near_pairs | equal_pairs; % pairs to remove

    nmask = [false, pair_nmask] | [pair_nmask, false]; % elements to remove
    bounds_out = bounds_in(~nmask);
end