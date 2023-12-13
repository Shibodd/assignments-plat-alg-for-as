function xdot_fn = bicycle_kin_glob_factory(lf, lr)
    function xdot_fn = bicycle_kin_glob(t, state, inputs)
        steering = inputs(1);
    
        heading = state(3);
        v = state(4);
    
        dx = v * cos(heading);
        dy = v * sin(heading);
        dheading = v * tan(steering) / (lf + lr);
        
        xdot_fn = [dx; dy; dheading; 0];
    end
    xdot_fn = @(t, x, u)(bicycle_kin_glob(t, x, u));
end

