function xdot_fn = bicycle_dyn_bodyalign_factory(m, I, lf, lr, cf, cr)
    wheelbase = lf + lr;

    function xdot = bicycle_dyn_glob(t, state, inputs)
        steering = inputs(1);
        v = inputs(2);

        momentum = m * v;
        iv = I * v;
        twocf = 2*cf;
        twocr = 2*cr;
        
        A = [-(twocf + twocr) / momentum, 0, -v - (twocf * lf - twocr * lr) / momentum;
             0, 0, 1;
             -(twocf * lf - twocr * lr) / iv, 0, -(twocf * lf * lf + twocr * lr * lr) / iv];

        B = [twocf / m; 0; twocf * lf / I];

        vy = state(3);
        phi = state(4);
        xdot = [v * cos(phi) - vy * sin(phi); % X in global frame
                v * sin(phi) + vy * cos(phi); % Y in global frame
                A * state(3:end) + B * steering % vy, phi, phidot
               ];
    end
    xdot_fn = @bicycle_dyn_glob;
end

