function xdot_fn = bicycle_kin_factory(lf, lr)
    wheelbase = lf + lr;

    function xdot_fn = bicycle_kin_glob(t, state, inputs)
        steering = inputs(1);
        v = inputs(2);
        
        heading = state(3);

        veh_slip = atan((lr * tan(steering)) / wheelbase);
        dx = v * cos(heading + veh_slip);
        dy = v * sin(heading + veh_slip);
        dheading = v * cos(veh_slip) * tan(steering) / wheelbase;
        
        xdot_fn = [dx; dy; dheading];
    end
    xdot_fn = @(t, x, u)(bicycle_kin_glob(t, x, u));
end

