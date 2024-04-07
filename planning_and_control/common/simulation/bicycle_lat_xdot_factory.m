function xdot_fn = bicycle_lat_xdot_factory(p, log_fn)
    arguments
        p VehicleParameters
        log_fn function_handle
    end
    
    function xdot = bicycle_lat_xdot(t, x, u, w)
        % x = [
        %   X(m)
        %   Y(m)
        %   Lateral Velocity(m/s)
        %   Heading(rad)
        %   Yaw Rate(rad/s)
        % ]
        % u = [ Steering Angle (rad) ]
        % w = [ Longitudinal Velocity (m/s) ]

        vx = w(1);
        
        A = [-(2*p.Cf + 2*p.Cr)/(p.M*vx), 0, -vx - (2*p.Cf*p.Lf - 2*p.Cr*p.Lr)/(p.M*vx);
             0, 0, 1;
             -(2*p.Cf*p.Lf - 2*p.Cr*p.Lr)/(p.I*vx), 0, -(2*p.Cf*p.Lf^2 + 2*p.Cr*p.Lr^2)/(p.I*vx)];

        B = [(2*p.Cf)/(p.M);
             0;
             (2*p.Lf*p.Cf)/(p.I)];
        
        vy = x(3);
        phi = x(4);
        xdot = [vx * cos(phi) - vy * sin(phi); % X in global frame
                vx * sin(phi) + vy * cos(phi); % Y in global frame
                A * x(3:end) + B * u(1) % vy, phi, phidot
               ];

        slip_angle = atan2(vy, vx);

        log_fn([
            t; % Time
            x; % X, Y, Lat Vel, Heading, Yaw Rate
            slip_angle; % Slip angle
            phi + slip_angle; % Course angle
            u(1); % Steering angle
            vx % Velocity
        ]);
    end
    xdot_fn = @bicycle_lat_xdot;
end