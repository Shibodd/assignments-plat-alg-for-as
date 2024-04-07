classdef testC < matlab.unittest.TestCase
    methods(Test)
        function [] = test_correctness(testCase)
            A = rand(4);
            B = rand(4,2);
            N = 10;
            u = rand(N, 1);
            x0 = rand(4,1);
            phidotdes = rand(1,N);

            Ti = Ti_fn_factory(A, B, N);
            c = c_factory(A, B, x0, phidotdes);
            
            import matlab.unittest.constraints.IsEqualTo
            import matlab.unittest.constraints.AbsoluteTolerance

            % Simulate the system with no initial conditions to verify Ti
            expected = x0;
            for i=1:N
                expected = A*expected + B(:,1)*u(i) + B(:,2)*phidotdes(i);
                actual = Ti(i)*u + c(:,i);
                verifyThat(testCase, actual, IsEqualTo(expected, "Within", AbsoluteTolerance(1e-6)));
            end
        end
    end
    
end