function [overlap] = simple_overlap_check(a, b)
    % Checks if two simple ranges overlap.
    arguments
        a (1,2)
        b (1,2)
    end

    overlap = b(1) < a(2) && b(2) > a(1);
end

