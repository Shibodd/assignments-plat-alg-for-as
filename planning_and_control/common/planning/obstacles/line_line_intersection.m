function tu = line_line_intersection(o1, t1, o2, t2)
    % Returns t, u such that o1 + t * t1 = o2 + u * t2

    assert(isequal(size(o1), size(o2)), "o1 and o2 must have the same size");
    assert(isequal(size(t1), size(t2)), "t1 and t2 must have the same size");

    A = [t1, -t2];

    % Check for unsolvable system
    if abs(det(A)) < eps
        tu = repelem(NaN, size(A, 1), size(o1, 2));
        return;
    end

    b = o2 - o1;
    tu = A \ b;
end