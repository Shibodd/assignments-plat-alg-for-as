function ctrl_fn = cl_ctrl_factory(K)
    function u = fn(x, xdes, ~, ~)
        u = -K * (x - xdes);
    end
    ctrl_fn = @fn;
end