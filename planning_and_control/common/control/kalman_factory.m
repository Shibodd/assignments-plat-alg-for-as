function [kf] = kalman_factory(x0, sigma0, measure_sigma, process_sigma)
    arguments
        x0 (4,1) double
        sigma0 double
        measure_sigma double
        process_sigma double
    end

    kf = KalmanFilter;
    kf.X = x0;
    kf.H = [
        1, 0, 0, 0; % Lateral deviation
        0, 0, 1, 0  % Heading deviation
    ];
    kf.P = eye(4) * sigma0;
    kf.Q = eye(4) * process_sigma;
    kf.R = eye(2) * measure_sigma;
end