function xdot_fn = bicycle_dyn_bodyalign_factory(m, I, lf, lr, cf, cr)
    wheelbase = lf + lr;

    function xdot_fn = bicycle_dyn_glob(t, state, inputs)
        steering = inputs(1);
        v = inputs(2);

        momentum = m * v;
        
        
        A = [0, 1, 0, 0;
            ]
        
        xdot_fn = [dx; dy; dheading];
    end
    xdot_fn = @bicycle_dyn_glob;
end

