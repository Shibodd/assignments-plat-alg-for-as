function [xs, ts] = discrete_ctrl_sim(x0, t0, t_max, ctrl_freq, xdot_fn, input_fn)
    t_step = 1 / ctrl_freq;
    xs = x0;
    ts = t0;
    
    t = t0;
    while t < t_max
        % Previous x and t
        x = xs(:, end);
        t = ts(end);

        % Input at previous t - note that we must compute this 
        % only once per step, outside ode45 
        % (the control is discrete)
        % (compute this inside ode45 and it blows up)
        u = input_fn(t, x);

        % Compute states and time until current time stamp.
        [tsol,xsol] = ode45(@(t,x) xdot_fn(t, x, u), [t t+t_step], x);

        % Store the computed state and timestamps
        xs = [xs xsol(1:end, :)'];
        ts = [ts tsol(1:end)'];
    end
end