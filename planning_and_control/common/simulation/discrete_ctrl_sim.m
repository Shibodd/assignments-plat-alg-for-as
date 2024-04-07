function [] = discrete_ctrl_sim(x, t, t_max, ctrl_interval, xdot_fn, ctrl_fn)
    while t < t_max
        % Input at previous t - note that we must compute this 
        % only once per step, outside ode45
        % (the control is discrete)
        u = ctrl_fn(t, ctrl_interval, x);

        % Integrate and ignore the states. Logging is taken care of by xdot_fn
        [ts,xs] = ode45(@(t, x) xdot_fn(t, x, u), [t t min(t+ctrl_interval, t_max)], x);

        x = xs(end,:)';
        t = ts(end);
    end
end