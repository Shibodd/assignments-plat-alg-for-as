function xdot_fn = full_xdot_factory(vp, lat_log_fn, long_log_fn)
    xdot_fn_lat = bicycle_lat_xdot_factory(vp, lat_log_fn);
    xdot_fn_long = long_xdot_factory(vp, long_log_fn);
    
    function xdot = full_xdot_fn(t, x, u)
        xdot = [
            xdot_fn_long(t, x(1:2), u(1));
            xdot_fn_lat(t, x(3:end), u(2:end), x(2))
        ];
    end
    xdot_fn = @full_xdot_fn;
end