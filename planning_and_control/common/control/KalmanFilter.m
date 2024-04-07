classdef KalmanFilter < handle
    properties
        X % Estimated state
        A % State transition
        B % Input effect
        H % Measurement-space to state-space transf.
        P % State covariance
        R % Measurement covariance
        Q % Process covariance
    end
    
    methods
        function [] = predict(obj, u)
            obj.X = obj.A * obj.X + obj.B * u;
            obj.P = obj.A * obj.P * obj.A' + obj.Q;
        end
        
        function [] = update(obj, z)
            y = z - obj.H * obj.X;

            S = obj.H * obj.P * obj.H' + obj.R;
            K = obj.P * obj.H' * inv(S);

            obj.X = obj.X + (K * y);
            obj.P = (eye(size(obj.X)) - K * obj.H) * obj.P;

            m = max(max(obj.P));
            if m > 1e2
               fprintf("Covariance matrix is exploding (max %.1f)!\n", m);
            end
        end
    end
end

