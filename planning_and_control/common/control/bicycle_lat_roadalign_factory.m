function sys = bicycle_lat_roadalign_factory(p, ctrlInterval, vx)
    arguments
        p VehicleParameters
        ctrlInterval double
        vx double {mustBeGreaterThan(vx, 1)}
    end


    
    % x = [
    %   Lateral Deviation (m);
    %   Lateral Deviation Rate (m/s);
    %   Heading Deviation (rad);
    %   Heading Deviation Rate (rad/s);
    % ]

    % u = [
    %   Steering Angle (rad)
    %   Path Angular Velocity (rad/s)
    % ]

    A = [0, 1, 0, 0;
         0, -(2*p.Cf + 2*p.Cr) / (p.M*vx), (2*p.Cf+2*p.Cr) / (p.M), (-2*p.Cf*p.Lf + 2*p.Cr*p.Lr) / (p.M*vx);
         0, 0, 0, 1;
         0, -(2*p.Cf*p.Lf - 2*p.Cr*p.Lr) / (p.I*vx), (2*p.Cf*p.Lf - 2*p.Cr*p.Lr) / (p.I), -(2*p.Cf*p.Lf^2 + 2*p.Cr*p.Lr^2) / (p.I * vx)];
    B_steering = [0;
         2*p.Cf / p.M;
         0;
         2*p.Lf*p.Cf / p.I
    ];
    B_desired = [0;
          -(2*p.Cf*p.Lf - 2*p.Cr*p.Lr) / (p.M*vx) - vx;
          0;
          -(2*p.Cf*p.Lf^2 + 2*p.Cr*p.Lr^2) / (p.I*vx)
    ];

    % Just assume the velocity is constant => LTI system
    
    % Combine the textbook matrices (and inputs)
    B = [B_steering, B_desired];

    % y = [Lateral Deviation; Heading Deviation]
    C = [1, 0, 0, 0;
         0, 0, 1, 0];

    % No feed-forward
    D = zeros(2);
    
    % Assume the actuator instantly reaches the steering angle and holds it => zero-order hold
    % Discretize the system
    sys = ss(A, B, C, D, ...
        'StateName', { 'LD', 'LDr', 'HD', 'HDr' }, ...
        'OutputName', { 'LD', 'HD' }, ...
        'InputName', { 'Steer', 'Phidotdes' } ...
    );
    sys = c2d(sys, ctrlInterval, 'zoh');
end