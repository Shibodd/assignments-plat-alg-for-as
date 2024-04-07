function xs = input2frenet(A, B, x0, phidotdes, us)
    N = size(phidotdes, 2);
    xs = zeros(4, N+1);
    xs(:,1) = x0;
    for i=2:N+1
        xs(:,i) = A*xs(:,i-1) + B(:,1)*us(i-1) + B(:,2)*phidotdes(i-1);
    end
    xs = xs(:,2:end);
end