function ctrl_fn = full_ctrl_factory(lat_ctrl_fn, long_ctrl_fn)
    function u = full_ctrl_fn(t, dt, x)
        ulong = long_ctrl_fn(t, dt, x(1:2));
        ulat = lat_ctrl_fn(t, dt, x(3:end), x(2));
        u = [ulong; ulat];
    end
    ctrl_fn = @full_ctrl_fn;
end