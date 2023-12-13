function [xs, ts] = discrete_ctrl_sim(x0, t0, t_max, ctrl_freq, xdot_fn, input_fn)
    t_step = 1 / ctrl_freq;
    steps = ceil(t_max / t_step);
    xs = [x0, zeros(size(x0, 1), steps)];
    ts = [t0, zeros(1, steps)];
    
    t = t0;
    i = 2;
    while t < t_max
        % Previous x and t
        x = xs(:, i - 1);
        t = ts(i - 1);

        % Input at previous t - note that we must compute this 
        % only once per step, outside ode45 
        % (the control is discrete - 
        % also, computing this inside ode45 will produce wrong results)
        u = input_fn(t, x);

        % Compute current state and time. Pass at least 3 timestamps
        % so that ode45 only returns x at those timestamps
        [tsol,xsol] = ode45(@(t,x) xdot_fn(t, x, u), [t t t+t_step], x);

        % Store the current state and time
        xs(:,i) = xsol(end, :)';
        ts(i) = tsol(end);

        i = i + 1;
    end
end