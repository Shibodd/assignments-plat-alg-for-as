function u = test_lat_ctrl_fn(x, ~, ~)
    arguments
        x (4,1) double
        ~, ~
    end

    % x = [
    %   Lateral Deviation (m);
    %   Lateral Deviation Rate (m/s);
    %   Heading Deviation (rad);
    %   Heading Deviation Rate (rad/s);
    % ]
    raw = -10e-2 * (x(1) + x(2));
    SAT = deg2rad(30);
    u = max(-SAT, min(SAT, raw));
end

