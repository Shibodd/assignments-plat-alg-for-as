function input_fn = sinsteer_factory(frequency, amplitude_rad)
    input_fn = @(t,x)(sin(2 * pi * frequency * t) * amplitude_rad);
end