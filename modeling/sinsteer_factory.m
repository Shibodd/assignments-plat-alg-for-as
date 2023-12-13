function getu = sinsteer_factory(frequency, amplitude_rad)
    getu = @(t,x)(sin(2 * pi * frequency * t) * amplitude_rad);
end