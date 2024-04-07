function Ti_fn = Ti_fn_factory(A, B, n)
    arguments
        A (:,:)
        B (:,2)
        n int32
    end
    
    m = size(A, 1);
    assert(isequal(size(A), [m, m]), "A must be square.");
    assert(isequal(size(B), [m, 2]), "B must m x 2.");

    function Ti = Ti(i)
        Ti = zeros(4, n);
    
        Ti(:,i) = B(:,1);
        for i=i-1:-1:1
            Ti(:,i) = A*Ti(:,i+1);
        end
    end
    Ti_fn = @Ti;
end