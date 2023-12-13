function input_fn = input_fn_composer(functions)
    input_fn = @(t, x) cellfun(@(f) f(t, x), functions);
end

