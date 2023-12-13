function input_fn = constant_input_factory(val)
    input_fn = @(t, x) val;
end

