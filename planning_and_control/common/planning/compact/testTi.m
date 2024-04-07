classdef testTi < matlab.unittest.TestCase
    methods(Test)
        function [] = test_correctness(testCase)
            A = rand(4);
            B = rand(4,2);
            N = 10;
            u = rand(N, 1);
            Ti = Ti_fn_factory(A, B, N);
            
            import matlab.unittest.constraints.IsEqualTo
            import matlab.unittest.constraints.AbsoluteTolerance

            % Simulate the system with no initial conditions to verify Ti
            expected = zeros(4,1);
            for i=1:N
                expected = A*expected + B(:,1)*u(i);
                actual = Ti(i) * u;
                verifyThat(testCase, actual, IsEqualTo(expected, "Within", AbsoluteTolerance(1e-6)));
            end
        end
    end
    
end