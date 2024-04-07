function c = c_factory(A, B, x0, phidotdes)
    arguments
        A (:,:)
        B (:,2)
        x0 (:,1)
        phidotdes (1,:)
    end
    
    n = size(phidotdes, 2);
    m = size(A, 1);
    assert(isequal(size(A, 2), m), "A must be square.");
    assert(isequal(size(B), [m, 2]), "B must m x 2.");
    assert(isequal(size(x0), [m, 1]), "x0 must be m x 1");
    assert(isequal(size(phidotdes), [1, n]), "phidotdes must be 1 x n")
    
    if n < 1
        c = [];
        return;
    end

    c = zeros(m, n);
    c(:,1) = A*x0 + B(:,2)*phidotdes(1);
    for i=2:n
        c(:,i) = A*c(:,i-1) + B(:,2)*phidotdes(i);
    end
end

