function xdot_fn = long_xdot_factory(vp, log_fn)
    function xdot = long_xdot(t, x, u)
        % x = [
        %   wheel_angvel;
        %   velocity
        % ]
        % u = [
        %   torque_in
        % ]
        
        w = x(1);
        v = x(2);

        % Longitudinal force due to slip
        wheelspeed = vp.WheelRadius * w;
        slip = slip_ratio(v, wheelspeed);
        Flong = 4 * vp.Clong * slip;

        % Aerodynamic resistance
        air_density = 1.184;
        Faero = air_density * vp.CdA * v^2;

        % Rolling resistance
        g = 9.81;
        Froll = vp.Croll * vp.M * g;

        acceleration = (Flong - Faero) / vp.M;

        log_fn([
            t;
            x;
            slip;
            acceleration;
            u
        ]);

        xdot = [
          (-(Flong + Froll) * vp.WheelRadius + 4 * u) / (4 * vp.WheelInertia);
          acceleration
        ];
    end

    xdot_fn = @long_xdot;
end